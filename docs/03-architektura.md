# 03. Architektura

## Struktura repozytorium

```
.
├── drone/            # firmware StampFly (git subtree z m5stack/M5StampFly)
│   ├── src/          #   kod upstreamu + nasze „haczyki” oznaczone [POSHOLD]
│   ├── lib/poshold/  #   NASZ kod niezależny od sprzętu (estymator, regulator, pomocnicze)
│   └── test/         #   testy jednostkowe `native`
├── joystick/         # firmware pilota (git subtree z m5stack/Atom-JoyStick/examples/StampFlyController)
├── tools/            # skrypty Pythona: telemetria, wykresy, analiza logów
├── docs/             # ta dokumentacja
└── .github/workflows # CI: kompilacja obu projektów + testy native
```

Zasada: nasze zmiany żyją w osobnych plikach (`lib/poshold/`, nowe pliki w `src/`),
a w plikach upstreamu są tylko krótkie wywołania oznaczone komentarzem `[POSHOLD]`.
Dzięki temu `git subtree pull` z upstreamu daje konflikty tylko w tych miejscach.

## Firmware drona (stan upstreamu)

### Pętla główna

Timer sprzętowy co 2500 µs ustawia flagę; `loop_400Hz()` czeka na nią aktywnie,
więc cała logika sterowania działa z częstotliwością **400 Hz** w jednym wątku.

```mermaid
flowchart TD
    T["Timer 400 Hz"] --> SR["sensor_read()<br/>IMU, Madgwick, ToF, Kalman wysokości, INA3221"]
    SR --> LED["led_drive()"]
    LED --> SM{"Mode"}
    SM -->|INIT| AVG["AVERAGE: kalibracja żyroskopu (800 próbek = 2 s)"]
    SM -->|PARKING| PK["silniki stop, reset filtrów,<br/>czeka na przycisk ARM"]
    SM -->|FLIGHT| FL["get_command() → angle_control() → rate_control()"]
    SM -->|AUTO_LANDING| AL["auto_landing() → angle_control() → rate_control()"]
    SM -->|FLIP| FP["flip()"]
    FL --> TM["telemetry() (co 10. obieg = 40 Hz, ESP-NOW)"]
    AL --> TM
    PK --> TM
```

### Częstotliwości

| Element | Częstotliwość | Uwagi |
|---|---|---|
| Pętla sterowania, IMU (BMI270, SPI 8 MHz), filtr Madgwicka | 400 Hz | |
| Dolny ToF (VL53L3, tryb MEDIUM, budżet 33 ms) | ~30 Hz | odczyt po przerwaniu INT |
| Filtr Kalmana wysokości | 400 Hz | predykcja z akcelerometru, korekcja ToF |
| Telemetria ESP-NOW | 40 Hz | pakiet 120 B |
| Pętla pilota | 100 Hz | timer 10 ms |

### Estymacja

- **Orientacja:** filtr Madgwicka (tylko IMU, bez magnetometru) — przechylenie i pochylenie
  są stabilne, **kąt yaw dryfuje** (całkowanie żyroskopu).
- **Układ osi drona** (konwencja lotnicza, po transformacji osi BMI270 w `sensor.cpp`):
  X — do przodu, Y — w prawo, Z — w dół. Roll dodatni = prawe skrzydło w dół,
  pitch dodatni = nos w górę, yaw dodatni = obrót w prawo (patrząc z góry).
- **Wysokość:** odległość z dolnego ToF (z odrzucaniem skoków > 500 mm) → filtr
  dolnoprzepustowy → trójstanowy filtr Kalmana `Alt_kalman` (wysokość, prędkość
  pionowa, bias akcelerometru Z). Brak kompensacji przechyłu odczytu ToF.
- **Napięcie baterii:** INA3221 kanał 2, filtrowane; próg niskiego napięcia 3,34 V
  (100 kolejnych odczytów) — w upstreamie **tylko zmienia kolor LED**.

### Kaskada regulatorów

```mermaid
flowchart LR
    ST["Drążki"] -->|Stable| AR["zadany kąt<br/>(limit 30°)"]
    AR --> APID["PID kąta<br/>(phi_pid, theta_pid)"]
    APID --> RR["zadana prędkość kątowa"]
    ST -->|Sport| RR
    RR --> RPID["PID prędkości kątowej<br/>(p_pid, q_pid, r_pid)"]
    RPID --> MIX["mikser silników"]
    THR["drążek gazu"] -->|Auto Alt| AREF["Alt_ref (0,05–1,8 m)"]
    AREF --> ALT["alt_pid → Z_dot_ref"]
    ALT --> ZD["z_dot_pid → ciąg wokół trymu<br/>(±15%)"]
    ZD --> MIX
    THR -->|Manual Alt| MIX
```

### Bezpieczeństwo w upstreamie

| Sytuacja | Zachowanie upstreamu |
|---|---|
| Brak pakietu od pilota > 100 ms (40 obiegów) | natychmiast `AUTO_LANDING` |
| Przeciążenie > 2 g | silniki stop (`PARKING`) |
| Wysokość > 2,0 m lub brak odczytu ToF w Auto Alt | stopniowe zmniejszanie ciągu |
| Niskie napięcie | tylko kolor LED (cyjan) |
| Wyłączenie silników z pilota | przycisk ARM w locie → auto-lądowanie; ponowny w trakcie lądowania → stop |

Zmiany wprowadzane przez ten projekt są opisane w [06-position-hold.md](06-position-hold.md)
i w ADR-ach.

## Firmware pilota (stan upstreamu)

- Pętla 100 Hz: odczyt STM32 po I2C (`joy_update()`), złożenie pakietu 25 B,
  wysłanie ESP-NOW do zapisanego drona.
- Interfejs: LVGL 8.3 (osobne zadanie FreeRTOS), dwa ekrany: **startowy** (wybór trybów)
  i **lotu** (sztuczny horyzont, start/lądowanie).
- Odbiór telemetrii drona (pakiety `88 88`) do wyświetlania kątów, napięcia, wysokości.
