# 09. Rozwiązywanie problemów

Najpierw zawsze: log telemetrii (`tools/telemetry_logger.py`) i wykresy
(`tools/plot_log.py`). Kody przyczyn (`ph_reason`, ekran pilota):
[04-protokol-espnow.md](04-protokol-espnow.md#dron--pilot-status-8-b-5-hz).

## Parowanie i łączność

| Objaw | Prawdopodobna przyczyna | Co zrobić |
|---|---|---|
| pilot pokazuje „No StampFly” | brak parowania lub inny kanał | sparuj ponownie (02-instalacja-i-wgrywanie.md); reset drona w trakcie parowania pilota |
| `DRONE FW?` / `PROTO ERR` | na dronie i pilocie firmware z różnych gałęzi / fabryczny | wgraj oba firmware'y z tej samej gałęzi |
| dron miga czerwono-niebiesko i piszczy | pilot z innym protokołem | jw. |
| częste `STB LINK`, zawisy bez powodu | zakłócenia 2,4 GHz, słaba bateria pilota | zmień miejsce, naładuj pilota, nie zasłaniaj anteny AtomS3 |
| logger: duże `lost` przez pilota | normalne przy obciążonym radiu (ok. 16 Hz) | do strojenia używaj logów z USB drona na uwięzi |
| logger przez pilota: kilka ramek na starcie, potem cisza, brak schematu | zbyt mały bufor USB-CDC pilota (256 B < ramka 236 B) | firmware pilota z `setTxBufferSize(2048)` w `setup()` |

## Optical flow

| Objaw | Prawdopodobna przyczyna | Co zrobić |
|---|---|---|
| Przy starcie `PMW3901 not found` | czujnik nie odpowiada na SPI | sprawdź montaż płytki czujnika; ⚠️ pin CS (G12) do weryfikacji ze schematem |
| `flow_squal` stale niskie (< 30), `flow_reject` = 4 | jednolita, błyszcząca lub ciemna podłoga | lepsze oświetlenie, mata z wyraźnym wzorem (np. puzzle piankowe, dywan we wzory) |
| `flow_reject` = 2 lub 3 | wysokość poza oknem 0,10–1,50 m | lataj w oknie lub zmień `kFlowMinHeight/MaxHeight` |
| `flow_reject` = 1 przy poprawnym czujniku | brak pomiaru ToF (zbyt wysoko, ciemna/pochłaniająca powierzchnia) | sprawdź `tof_raw_mm` w telemetrii |
| `vel_x/y` rośnie przy pochylaniu drona w miejscu | błędny znak osi **albo zawyżona skala** (nachylenie `vel_x` vs `h·q` ≈ +1) | skala z pochyleń (08-testy.md, pkt 10a); znak osi z przeniesienia |
| `flow_dx = flow_dy = −365`, `flow_squal = 147`, `flow_shutter = 37886` stale | pierwszy burst po transakcji BMI270 nie dociera do czujnika (wzór `FF FE 93 FE…`) | `pmw3901_read_motion()` wykonuje burst dwa razy; diagnostyka: build z `-DPOSHOLD_PMW_DEBUG=1`, linie `[PMWDBG]` |
| `flow_shutter = 8191` stale (maksimum) | czujnik na pełnej ekspozycji: słabe światło | lepsze oświetlenie podłogi |
| `pos_x/y` po przeniesieniu o 1 m ≠ 1 m | błędna skala `kFlowRadPerCount` lub wysokość | powtórz kalibrację na stałej wysokości |
| `vel_x/y` dryfuje powoli przy nieruchomym dronie | duży bias akcelerometru; dron nieskalibrowany na poziomej powierzchni | uruchamiaj drona na płaskim, poziomym podłożu (kalibracja „fioletowa”) |

## Position Hold w locie

| Objaw | Prawdopodobna przyczyna | Co zrobić |
|---|---|---|
| Position Hold nie włącza się, pilot `STB NO CAL` | `kFlowAxesVerified = false` | wykonaj kalibrację flow |
| `STB NO SENS` | PMW3901 niewykryty przy starcie | zob. tabela „Optical flow” |
| po starcie 3 piknięcia i `STB FLOW` | słaba tekstura/światło przy starcie | lepsza podłoga, sprawdź `flow_squal` |
| dron powoli dryfuje mimo Position Hold | błąd skali flow, dryf yaw, przechylona podłoga | powtórz kalibrację; sprawdź `pos_x/y` vs rzeczywistość; start na poziomej powierzchni |
| dron oscyluje wokół punktu | za duże `kVelKp`/`kPosKp` | 07-strojenie.md |
| dron „ucieka” coraz szybciej w jedną stronę | **odwrotny znak** osi flow (dodatnie sprzężenie) | natychmiast KILL / Stable; powtórz kalibrację i test „w ręku” |
| `STB FENCE` i lądowanie | dron > 2,5 m od startu (zwykle błąd estymacji) | sprawdź kalibrację; ewentualnie zwiększ `kFenceRadius` |
| `STB HEIGHT` | wysokość poza 0,10–1,50 m | lataj niżej / wyżej; dostosuj okno |

## Krótkie loty

| Objaw | Prawdopodobna przyczyna | Co zrobić |
|---|---|---|
| auto-lądowanie `STB BATT` po < 2 min | próg `kBatteryLandVolt` za wysoki dla Twojej baterii | sprawdź `vbat_filt_v` w logu, skoryguj próg (07-strojenie.md) |
| krótkie loty mimo naładowanej baterii | zużyta bateria LiHV (wysoki opór wewnętrzny), uszkodzone śmigła | wymień baterię/śmigła; ładuj w pilocie do 4,35 V |
| bateria nie ładuje się w pilocie | kabel bez zasilania, dioda czerwona = ładowanie | użyj zasilacza 5 V/1 A; zielona dioda = naładowana |

## Lot na uwięzi

| Objaw | Przyczyna | Co zrobić |
|---|---|---|
| w Stable dron znosi ~0,5 m/s bez drążków | brak trzymania pozycji, offset przechylenia ~1° | to nie błąd; testuj Position Hold z ciasną uwięzią |
| przy Auto Alt dron wznosi się wyżej niż zadana (~0,2 m) | przeregulowanie fabrycznego regulatora | opuść zadaną wysokość lewym drążkiem po starcie |
| auto-lądowanie po kilku sekundach lotu | bateria < 4,0 V w spoczynku, spadek pod obciążeniem do progu 3,35 V | naładuj do 4,3 V przed lotem |
| `p` skacze do ±250 °/s, silniki stają | uderzenie lub szarpnięcie uwięzi; zabezpieczenie przeciążeniowe upstreamu | skróć/usztywnij uwięź, usuń przeszkody |

## Build i narzędzia (Windows)

| Objaw | Przyczyna | Co zrobić |
|---|---|---|
| `error: looser throw specifier` w `FastLED/.../fs_sdcard_arduino.cpp.hpp` | `FS.h` znajdowany jako `fs.h` na Windows | `lib_ignore = SD` w `platformio.ini` (już jest) |
| `logs\...` zapisuje się w złym katalogu | logger zapisuje względem katalogu bieżącego | uruchamiaj z katalogu głównego repozytorium lub podaj `--out` |
| `thrust_duty` zostaje stałe po KILL | telemetria pokazuje ostatni `Thrust_command` (nie zerowany w parkowaniu); napięcie baterii wraca, silniki stoją | do poprawy w telemetrii |
