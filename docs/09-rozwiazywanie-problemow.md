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
| logger: duże `lost` przez pilota | normalne przy obciążonym radiu (25 Hz) | do strojenia używaj logów z USB drona na uwięzi |

## Optical flow

| Objaw | Prawdopodobna przyczyna | Co zrobić |
|---|---|---|
| Przy starcie `PMW3901 not found` | czujnik nie odpowiada na SPI | sprawdź montaż płytki czujnika; ⚠️ pin CS (G12) do weryfikacji ze schematem |
| `flow_squal` stale niskie (< 30), `flow_reject` = 4 | jednolita, błyszcząca lub ciemna podłoga | lepsze oświetlenie, mata z wyraźnym wzorem (np. puzzle piankowe, dywan we wzory) |
| `flow_reject` = 2 lub 3 | wysokość poza oknem 0,10–1,50 m | lataj w oknie lub zmień `kFlowMinHeight/MaxHeight` |
| `flow_reject` = 1 przy poprawnym czujniku | brak pomiaru ToF (zbyt wysoko, ciemna/pochłaniająca powierzchnia) | sprawdź `tof_raw_mm` w telemetrii |
| `vel_x/y` rośnie przy pochylaniu drona w miejscu | błędny znak/zamiana osi flow | powtórz kalibrację (08-testy.md) |
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
