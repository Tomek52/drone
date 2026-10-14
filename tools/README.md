# tools/

Skrypty Pythona do pracy z dronem na komputerze.

```powershell
py -m pip install -r tools/requirements.txt
```

| Skrypt | Do czego |
|---|---|
| `telemetry_logger.py` | zapis telemetrii z portu USB (dron albo pilot) do `logs/*.csv` |
| `plot_log.py` | wykresy z pliku CSV (PNG obok CSV, opcjonalnie okno) |
| `flow_calibration.py` | mapowanie osi i skala optical flow z próby przenoszenia (docs/08-testy.md) |
| `stampfly_telemetry.py` | biblioteka: dekoder ramek binarnych (używana przez powyższe) |
| `test_*.py` | testy (`py -m unittest discover -s tools -v`) |

## Zapis telemetrii

Dwa źródła, ten sam format:

| Źródło | Kiedy | Częstotliwość |
|---|---|---|
| USB drona | stół, uwięź (dron podłączony kablem) | 50 Hz |
| USB pilota | prawdziwy lot (pilot przekazuje dane z ESP-NOW) | 25 Hz |

```powershell
py tools/telemetry_logger.py --list            # lista portów, ESP32-S3 oznaczone
py tools/telemetry_logger.py --port COM5       # zapis do logs/RRRRMMDD_GGMMSS.csv
py tools/telemetry_logger.py --port COM5 --raw --duration 120
py tools/telemetry_logger.py --from-raw logs/20261014_120000.bin   # ponowne dekodowanie
```

W trakcie zapisu co sekundę pojawia się linia stanu: liczba wierszy, zgubione
ramki (`lost`), błędne sumy kontrolne (`bad`) oraz kilka kluczowych wartości.
Pierwsze dane pojawiają się po odebraniu schematu (najpóźniej po 2 s).

Port COM drona i pilota to natywne USB ESP32-S3 — tylko jeden program naraz może
mieć port otwarty (zamknij monitor portu w VS Code przed uruchomieniem loggera).

## Wykresy

```powershell
py tools/plot_log.py logs/20261014_120000.csv
py tools/plot_log.py logs/20261014_120000.csv --groups attitude altitude --t0 5 --t1 25 --show
```

Grupy paneli: `attitude`, `rates`, `altitude`, `position`, `velocity`, `flow`,
`poshold`, `battery`, `sticks`, `timing`. Panele bez danych w logu są pomijane.
Konwencja: niebieska linia ciągła = pomiar/estymata, pomarańczowa przerywana =
wartość zadana, cienka zielona = surowy czujnik.

## Pola telemetrii

Lista pól jest w `drone/src/telemetry_stream.cpp` (tablica `kFields`). Jednostka
jest częścią nazwy (`_deg`, `_dps`, `_m`, `_mps`, `_v`). Dodanie pola w firmware
nie wymaga zmian w skryptach.
