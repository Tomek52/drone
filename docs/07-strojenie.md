# 07. Strojenie regulatorów

Wszystkie parametry Position Hold są w `drone/lib/poshold/src/poshold_config.hpp`.
Po każdej zmianie: `pio run -d drone -t upload` i nowy log.

## Narzędzia

1. Zapis logu: `py tools/telemetry_logger.py --port COMx` (port pilota w locie,
   port drona na stole/uwięzi) — zob. [tools/README.md](../tools/README.md).
2. Wykresy: `py tools/plot_log.py logs/<plik>.csv --groups <grupy>`.
3. Jeden log = jedna zmiana parametru. Zapisuj w notatce: nazwa pliku logu,
   zmieniony parametr, wartość, obserwacje.

## Ogólna kolejność (od wewnątrz kaskady na zewnątrz)

Regulator zewnętrzny stroimy dopiero, gdy wewnętrzny działa dobrze:

1. prędkość kątowa (`rates`) — fabrycznie dostrojona przez M5Stack/Kouhei Ito,
   zwykle nie ruszamy;
2. kąt (`attitude`) — fabrycznie dostrojony;
3. wysokość (`altitude`) — fabrycznie dostrojona dla trybu Auto Altitude;
4. prędkość pozioma i pozycja (`velocity`, `position`) — nasze, stroimy od zera.

## Na co patrzeć na wykresach

| Objaw | Interpretacja | Typowa reakcja |
|---|---|---|
| pomiar oscyluje wokół wartości zadanej z ustaloną częstotliwością | za duże wzmocnienie P (lub D przy szybkich oscylacjach) | zmniejsz P o 20–30% |
| pomiar wolno dochodzi do wartości zadanej, bez przeregulowania | za małe P | zwiększ P o 20–30% |
| stały uchyb, który powoli maleje | za słabe I | zwiększ I (w upstreamie: **zmniejsz** `ti`, bo to stała czasowa) |
| powolne „pływanie” po zwiększeniu I | za duże I | zmniejsz I |
| szum w wartości zadanej wyższego poziomu | za duże D lub zaszumiony czujnik | zmniejsz D / zwiększ filtrację |

Uwaga: w upstreamie regulator PID ma postać `kp, ti, td` (stała całkowania `ti` —
**im większa, tym słabsze całkowanie**).

## Strojenie Position Hold krok po kroku

Warunki: kalibracja optical flow zakończona (`kFlowAxesVerified = true`,
[08-testy.md](08-testy.md#kalibracja-optical-flow-przenoszenie-w-ręku)), teksturowana,
jasna podłoga, logger na porcie **pilota**, wysokość lotu ~0,5 m.

### Krok 1 — estymator (bez sterowania)

Lot w **Stable + Auto Alt**, kilka powolnych ruchów przód/tył i na boki.
`plot_log.py --groups velocity flow`:

- `vel_x_mps` (estymata) powinna nadążać za `flow_vx_mps` (surowe) bez opóźnienia
  większego niż ~0,1 s i z dużo mniejszym szumem.
- Duże opóźnienie → zmniejsz `kVelCorrectionTau` (np. 0,15 → 0,10).
- Duży szum estymaty → zwiększ `kVelCorrectionTau` (0,15 → 0,25).
- `flow_valid` powinno być stale 1; częste zera → sprawdź `flow_squal` i `flow_reject`.

### Krok 2 — pętla prędkości (`kVelKp`, `kVelKi`)

Włącz Position Hold, po ustabilizowaniu daj krótkie pełne wychylenie prawego drążka do
przodu (~1 s) i puść. `plot_log.py --groups velocity poshold`:

| Obserwacja | Zmiana |
|---|---|
| `vel_x_mps` oscyluje wokół `vel_sp_x_mps` (falowanie ~1–3 Hz) | zmniejsz `kVelKp` o 25% |
| prędkość wolno dochodzi do zadanej, dron „ślizga się” po puszczeniu drążka | zwiększ `kVelKp` o 25% |
| stała różnica prędkości / powolne „pływanie” w jedną stronę | zwiększ `kVelKi` (0,5 → 0,8) |
| wolne kołysanie (okres kilka sekund) | zmniejsz `kVelKi` |
| kąty zadane często na limicie ±10° | dron nie nadąża — zmniejsz `kMaxSpeed` lub zwiększ `kMaxTilt` (ostrożnie, max 15°) |

### Krok 3 — pętla pozycji (`kPosKp`)

Zawis bez drążków przez 30 s. `plot_log.py --groups position poshold`:

| Obserwacja | Zmiana |
|---|---|
| dron wraca do punktu zadanego zbyt wolno, „błądzi” ±20 cm | zwiększ `kPosKp` (1,0 → 1,5) |
| przeregulowanie po puszczeniu drążka, oscylacja pozycji | zmniejsz `kPosKp` (1,0 → 0,7) |
| dron zatrzymuje się dalej niż oczekiwano po puszczeniu drążka | zmniejsz `kBrakeTau` |
| dron „cofa się” po puszczeniu drążka | zwiększ `kBrakeTau` |

Zasada: `kPosKp` co najmniej 3–4 razy mniejsze niż pasmo pętli prędkości — jeśli trzeba
mocno zwiększyć `kPosKp`, najpierw popraw pętlę prędkości.

### Krok 4 — progi bezpieczeństwa

- **Bateria:** z kilku pełnych lotów (`--groups battery`) odczytaj napięcie filtrowane
  `vbat_filt_v` przy zawisie tuż przed tym, jak dron przestaje utrzymywać wysokość.
  Ustaw `kBatteryLandVolt` ok. 0,1 V wyżej, `kBatteryWarnVolt` ok. 0,25 V wyżej.
- **SQUAL:** `kFlowMinSqual` ok. 50–60% typowej wartości nad Twoją podłogą.
- **Okno wysokości:** jeśli powyżej ~1 m `flow_squal` mocno spada, obniż `kFlowMaxHeight`
  i `kMaxAltitude`.

### Szablon notatki

| Log | Parametr | Stara → nowa | Obserwacja |
|---|---|---|---|
| `logs/20261015_101500.csv` | `kVelKp` | 2,0 → 1,5 | mniejsze falowanie, hamowanie dłuższe o ~10 cm |

### Dziennik zmian parametrów (nasz sprzęt)

| Log | Parametr | Stara → nowa | Obserwacja |
|---|---|---|---|
| `logs/20261014_221210.csv` | `kFlowSwapXY` | `false` → `true` | ruch do przodu = licznik czujnika −Y, w prawo = +X |
| `logs/20261014_221210.csv` | `kFlowSignX` | 1,0 → −1,0 | jw. (po zamianie osi) |
| `logs/20261014_221210.csv` | `kFlowRadPerCount` | 0,0209 → 0,00341 | ok. 6× mniej niż z datasheetu (1 m na ~0,42 m ≈ 745 zliczeń). Przeciek między osiami 21% / 17% (próg 20%) — **do weryfikacji** powtórnym przeniesieniem; `kFlowAxesVerified` nadal `false` |
