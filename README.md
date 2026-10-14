# StampFly Position Hold

Firmware drona **M5Stack StampFly v1.0** i pilota **Atom JoyStick (AtomS3)**,
oparty na oficjalnym, otwartym firmware M5Stack (licencja MIT), rozszerzony
o nowy tryb lotu **Position Hold / Auto**: dron wisi w miejscu (XY z czujnika
optical flow PMW3901, Z z czujnika ToF), a drążki przesuwają punkt zadany.

> ⚠️ Projekt nie był jeszcze testowany w locie. Zanim polecisz, przejdź procedurę
> testów (niżej) od stołu bez śmigieł. Stan prac: [docs/CHANGELOG.md](docs/CHANGELOG.md).

## Tryby lotu

| Tryb | Przycisk na pilocie | Kolor LED drona |
|---|---|---|
| Stable | lewy przedni, cyklicznie (ekran startowy) | żółty (Manual Alt) / purpurowy (Auto Alt) |
| Sport | lewy przedni, cyklicznie (ekran startowy) | różowy `DC669B` |
| **Position Hold** | lewy przedni, cyklicznie (ekran startowy); w locie lewy przedni = powrót do Stable | **stały niebieski** (migający = utracony, lot w Stable) |
| **KILL** (wyłączenie silników) | **przytrzymaj wciśnięty lewy drążek 0,5 s** — w każdym trybie | — |

Prawy przedni przycisk: Auto ⇄ Manual Altitude (Position Hold zawsze używa Auto).
Ekran AtomS3: start / lądowanie. Wciśnięcie prawego drążka: flip (nie w Position Hold).

W Position Hold (Mode 2): **prawy drążek** przesuwa drona przód/tył/lewo/prawo w jego
układzie (pełne wychylenie = 0,5 m/s, po puszczeniu dron staje i trzyma pozycję),
**lewy drążek** — góra/dół i obrót. Opcjonalny tryb krokowy (30 cm na wychylenie)
i wszystkie limity: `drone/lib/poshold/src/poshold_config.hpp`.
Szczegóły: [docs/05-tryby-lotu.md](docs/05-tryby-lotu.md), [docs/06-position-hold.md](docs/06-position-hold.md).

## Wymagania

- **Sprzęt:** StampFly **v1.0** (K138, StampS3) i Atom JoyStick (K137) z **AtomS3** —
  nie wersje v1.1 (Stamp-S3A / AtomS3R). Bateria 1S LiHV 300 mAh.
- **Windows 11:** VS Code + rozszerzenie **PlatformIO IDE**, Git for Windows,
  Python 3.10+ (skrypty w `tools/`), opcjonalnie GCC (MSYS2) do testów `native`.
- **Sterowniki USB:** StampS3 i AtomS3 używają natywnego USB ESP32-S3 — Windows 11 ma
  sterownik wbudowany (⚠️ do potwierdzenia na Twoim komputerze).

Szczegóły: [docs/02-instalacja-i-wgrywanie.md](docs/02-instalacja-i-wgrywanie.md).

## Klonowanie, budowanie, testy

```powershell
git clone https://github.com/Tomek52/drone.git
cd drone
code stampfly.code-workspace      # otwiera oba projekty w VS Code

pio run -d drone                  # kompilacja firmware'u drona
pio run -d joystick               # kompilacja firmware'u pilota
pio test -d drone -e native       # testy jednostkowe na komputerze (bez drona)
py -m pip install -r tools/requirements.txt
py -m unittest discover -s tools  # testy skryptów Pythona
```

## Wgrywanie i parowanie

| Urządzenie | Tryb pobierania | Polecenie |
|---|---|---|
| **Dron** | odłącz baterię, **przytrzymaj środkowy przycisk na StampS3** i podłącz USB | `pio run -d drone -t upload` |
| **Pilot** | podłącz USB, **przytrzymaj boczny przycisk AtomS3 ~2 s**, aż zaświeci zielona dioda (krótkie wciśnięcie = reset) | `pio run -d joystick -t upload` |

**Wgraj zawsze oba firmware'y z tej samej gałęzi** — protokół ma numer wersji;
przy niezgodności dron nie wystartuje (LED miga czerwono-niebiesko, pilot: `DRONE FW?`).

**Parowanie:** przytrzymaj środkowy przycisk (ekran) AtomS3 i włącz pilota → wciśnij
ekran ponownie (tryb parowania) → naciśnij reset na StampFly → poczekaj na ekran sukcesu.

**Powrót do firmware'u fabrycznego:** M5Burner → typ urządzenia **STAMPS3** →
„StampFly Firmware” i „StampFly Controller Firmware” → tryb pobierania jak wyżej → Burn → parowanie.

Pełne instrukcje: [docs/02-instalacja-i-wgrywanie.md](docs/02-instalacja-i-wgrywanie.md).

## Ładowanie i bezpieczeństwo baterii LiHV

- Ładuj w slotach **CH1/CH2 pilota** po podłączeniu kabla USB (zasilacz 5 V/1 A, ~55 min).
  Dioda czerwona = ładowanie, zielona = naładowana do 4,35 V.
- Używaj tylko ładowarki do **LiHV (4,35 V)** — pilot ma taki układ (TP4067).
- Nie rozładowuj pod obciążeniem poniżej **3,0 V**. Firmware ląduje automatycznie przy
  niskim napięciu (progi do ustawienia z logów).
- Nie przechowuj naładowanej baterii dłużej niż 3 dni; do przechowywania **3,8–3,9 V**
  (pilot piszczy przy ~3,85 V na ekranie startowym).
- Wkładaj wtyk baterii ostrożnie i we właściwej orientacji — przesunięty wtyk może
  uszkodzić elektronikę i grozi pożarem baterii. Spuchniętej baterii nie używaj ani nie ładuj.
- Nie ładuj bez nadzoru; śmigła mogą się odczepić — używaj okularów ochronnych.

Źródła i szczegóły: [docs/01-sprzet.md](docs/01-sprzet.md#baterie--zasady-doc).

## Procedura testów (zawsze w tej kolejności)

1. **Stół, bez śmigieł:** telemetria, wykrycie PMW3901, **kalibracja optical flow**
   (przenoszenie w ręku), protokół i KILL, **test „w ręku”** kierunków regulatora.
2. **Uwięź:** Stable, potem Position Hold, KILL, wyłączenie pilota (zawis → lądowanie).
3. **Niski lot** (~0,5 m) nad **teksturowaną, dobrze oświetloną podłogą**, wolna przestrzeń 3 × 3 m.

Krok po kroku: [docs/08-testy.md](docs/08-testy.md).

## Strojenie regulatorów z logów

```powershell
py tools/telemetry_logger.py --port COM5           # port pilota w locie / drona na uwięzi
py tools/plot_log.py logs/<plik>.csv --groups velocity position poshold
```

Procedura (estymator → prędkość → pozycja → progi bezpieczeństwa):
[docs/07-strojenie.md](docs/07-strojenie.md).

## Rozwiązywanie problemów

Dryf, parowanie, jakość optical flow, krótkie loty, kody błędów na ekranie pilota:
[docs/09-rozwiazywanie-problemow.md](docs/09-rozwiazywanie-problemow.md).

## Co nie zostało potwierdzone w oficjalnych źródłach

Zweryfikowano z dokumentacją M5Stack: wersje sprzętu, mapę pinów (w tym CS PMW3901 = G12),
adresy I2C, procedury wgrywania, parowania i ładowania, M5Burner. **Do weryfikacji na sprzęcie:**

| Co | Gdzie opisane |
|---|---|
| orientacja montażu i skala PMW3901 (kalibracja obowiązkowa) | docs/06, docs/08 |
| sterownik USB w Windows 11 bez instalacji | docs/02 |
| czy dron startuje sam po wgraniu w trybie pobierania (zwykle trzeba zresetować) | docs/02 |
| czy parowanie przetrwa ponowne wgranie firmware'u pilota | docs/02 |
| adres wgrywania gotowych plików `.bin` z upstreamu (zalecany M5Burner) | docs/02 |
| wartość rezystora bocznikowego INA3221 (prąd baterii nie jest jeszcze mierzony) | docs/01 |
| fizyczne położenie przycisku KILL (wg kodu: wciśnięcie lewego drążka w Mode 2) | docs/05 |
| progi napięcia baterii, wzmocnienia regulatorów (wartości wstępne z symulacji) | docs/07 |

## Struktura repozytorium

| Katalog | Zawartość |
|---|---|
| `drone/` | firmware StampFly (upstream `m5stack/M5StampFly` + nasze zmiany oznaczone `[POSHOLD]`) |
| `drone/lib/poshold/` | nasz kod niezależny od sprzętu: estymator, regulator, nadzorca, konfiguracja |
| `joystick/` | firmware pilota (upstream `m5stack/Atom-JoyStick` + nasze zmiany) |
| `common/` | wspólne definicje dron ⇄ pilot ⇄ PC (protokół, ramki telemetrii) |
| `tools/` | skrypty Pythona: zapis telemetrii, wykresy, kalibracja optical flow |
| `docs/` | dokumentacja (po polsku) — zacznij od [docs/README.md](docs/README.md) |

Aktualizacja z upstreamu M5Stack (git subtree):
[docs/02-instalacja-i-wgrywanie.md](docs/02-instalacja-i-wgrywanie.md#aktualizacja-z-upstreamu-m5stack).

## Licencja

Kod upstreamu: MIT (M5Stack, Kouhei Ito) — zob. `drone/LICENSE`, `joystick/LICENSE`.
Sekwencja inicjalizacji PMW3901 wg sterownika Bitcraze (MIT). Nasze zmiany: MIT (`LICENSE`).
