# 08. Testy

## Testy jednostkowe na komputerze (`native`)

Kod niezależny od sprzętu leży w `drone/lib/poshold/` i jest kompilowany zarówno
na ESP32, jak i na komputerze. Testy (framework Unity) są w `drone/test/`.

```powershell
pio test -d drone -e native              # wszystkie testy
pio test -d drone -e native -f test_math_utils   # wybrany test
```

Na Windows potrzebny jest kompilator GCC w `PATH` (np. MSYS2 MinGW-w64) albo WSL —
zob. [02-instalacja-i-wgrywanie.md](02-instalacja-i-wgrywanie.md).

CI (GitHub Actions, `.github/workflows/ci.yml`) przy każdym PR:
1. kompiluje firmware drona (`esp32-s3-devkitc-1`),
2. kompiluje firmware pilota (`m5stack-atoms3`),
3. uruchamia `pio test -e native` dla drona,
4. uruchamia testy narzędzi Pythona.

| Test | Co sprawdza |
|---|---|
| `test_math_utils` | funkcje pomocnicze: ograniczanie, strefa martwa, zawijanie kąta, obrót 2D |
| `test_position_estimator` | estymator z symulatorem ruchu: znak i skala flow, kompensacja obrotu, wysokość z przechyłem, akcelerometr w układzie heading, bramki jakości z histerezą, estymacja biasu, przerwy flow, całkowanie pozycji z yaw, mapowanie osi |
| `test_telemetry_frame` | kodowanie ramek telemetrii, suma kontrolna, dzielenie schematu na fragmenty, wektory wzorcowe wspólne z Pythonem |

Testy narzędzi w Pythonie (dekoder telemetrii, kalibracja flow):

```powershell
py -m unittest discover -s tools -v
```

## Testy na sprzęcie

### Telemetria (stół, bez śmigieł)

1. Zdejmij śmigła. Podłącz drona kablem USB (bez trybu pobierania) i baterię.
2. `py tools/telemetry_logger.py --list`, potem `--port COMx` dla drona.
3. Po ≤ 2 s powinny pojawiać się wiersze; `lost` ≈ 0, `bad` = 0.
4. Pochyl drona ręcznie — `roll_deg`/`pitch_deg` na wykresie (`plot_log.py --groups attitude`)
   powinny się zgadzać ze znakami w [03-architektura.md](03-architektura.md)
   (roll dodatni = prawa strona w dół, pitch dodatni = nos w górę).
5. Podłącz pilota do PC, uruchom logger na porcie pilota, włącz drona na baterii —
   dane powinny płynąć przez ESP-NOW (25 Hz).


### Optical flow — wykrycie czujnika (stół, bez śmigieł)

1. Podłącz drona przez USB, otwórz monitor portu (`pio device monitor -d drone`) i zresetuj drona.
2. W komunikatach startowych musi się pojawić: `[POSHOLD] PMW3901 ready (id=0x49)`.
   - `PMW3901 not found (id=0x.. inv=0x..)` → czujnik nie odpowiada: sprawdź, czy płytka
     czujnika jest na miejscu; ⚠️ pin CS (G12 wg dokumentacji M5Stack) do weryfikacji.
3. Uruchom logger, trzymaj drona 0,3–0,5 m nad podłogą i obserwuj `flow_squal`
   (`plot_log.py --groups flow`). Zanotuj typowe wartości nad Twoją podłogą
   (dywan, panele, płytki) — próg `kFlowMinSqual` = 30 może wymagać korekty.

### Kalibracja optical flow (przenoszenie w ręku)

Cel: wyznaczyć mapowanie osi czujnika na osie drona i skalę. Wymagane przed
pierwszym włączeniem Position Hold.

1. **Śmigła zdjęte.** Dron na USB, `py tools/telemetry_logger.py --port COMx`.
2. Połóż na podłodze taśmę mierniczą. Podłoga teksturowana, dobrze oświetlona.
3. Trzymaj drona **poziomo**, na stałej wysokości ~0,3–0,5 m, bez obracania.
4. Odczekaj 3 s, przesuń drona **do przodu dokładnie o 1,00 m** (ok. 2–3 s), odczekaj 3 s.
5. Przesuń **w prawo dokładnie o 1,00 m**, odczekaj 3 s. Zatrzymaj logger (Ctrl+C).
6. `py tools/plot_log.py logs/<plik>.csv --groups flow velocity` — odczytaj czasy
   początku i końca obu ruchów (wykres `flow_dx/dy` lub `flow_vx/vy`).
7. `py tools/flow_calibration.py logs/<plik>.csv --forward T0 T1 1.0 --right T0 T1 1.0`
8. Wpisz wynik do `drone/lib/poshold/src/poshold_config.hpp`, zbuduj i wgraj firmware.
9. **Weryfikacja:** powtórz przeniesienie do przodu o 1 m — `pos_x_m` powinno wzrosnąć
   o ~1,0 (±15%), `vel_x_mps` dodatnie podczas ruchu; w prawo — `pos_y_m` ~+1,0.
10. **Test kompensacji obrotu:** nad jednym punktem pochylaj drona nosem w górę/dół
    i na boki (±15°) bez przesuwania. `vel_x_mps` i `vel_y_mps` powinny pozostać
    blisko zera (< 0,15 m/s). Duże wartości w fazie z pochylaniem = zły znak osi
    flow (powtórz kalibrację).
11. Dopiero po pozytywnej weryfikacji ustaw `kFlowAxesVerified = true`.

Pozostałe procedury (uwięź → niski lot) zostaną opisane w gałęzi
`feat/position-controller`.
