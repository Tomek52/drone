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
| `logs/20261014_221210.csv` | `kFlowRadPerCount` | 0,0209 → 0,00341 | z przenoszenia 1 m w ręku; odległości niewiarygodne (powtórzenie dało rozrzut ±40% między osiami), skorygowane w następnym wierszu |
| `logs/20261014_222429.csv` | `kFlowRadPerCount` | 0,00341 → 0,00175 | skala z czystych pochyleń względem kąta/żyroskopu (8 okien: 0,0015–0,0018, mediana 0,00175); znaki kompensacji obrotu zgodne z docs/06; regresja `vel_x` na `h·q` miała nachylenie +0,8 (powinno ≈ 0) |
| `logs/20261014_223324.csv` | `kFlowAxesVerified` | `false` → `true` | po K = 0,00175: nachylenia kompensacji −0,06 / +0,03 (przed +0,76 / −0,90), `vel_x` max 0,12, `vel_y` max 0,20 przy szybkim pochyleniu (p95 0,14); ToF na stoliku 35 cm = 347 mm. Dystans 1 m ±15% niezweryfikowany ręcznie — sprawdzić na uwięzi |
| `logs/20261014_235803.csv` | `z_dot_kp` (upstream, pętla prędkości pionowej) | 0,08 → 0,06 | Position Hold na uwięzi: XY dobrze (±7 cm), ale wysokość oscyluje 0,21–0,92 m wokół 0,50 m z okresem ~3,2 s, gaz uderza w ograniczenia ±15% (0,54–0,76), `vz` do 1,3 m/s. Zmieniony tylko `z_dot_kp`; w telemetrii pole `thrust0` (zastąpiło `acc_bias_y`). Oczekiwanie: mniejsza amplituda, brak uderzeń w ograniczenia; jeśli nie — następnie `alt_kp` |
| `logs/20261015_001652.csv` | `z_dot_kp` (wynik) | 0,06 | wysokość 0,38–0,71 m (było 0,21–0,92), std. 0,080 m (było 0,125), gaz 0,67–0,73 poza ograniczeniami ±15%, `vz` std. 0,12 (było 0,33), błąd XY 4 cm. Amplituda rośnie w drugiej połowie lotu razem ze spadkiem napięcia 4,09 → 3,61 V (`thrust0` 0,67 → 0,71): wpływ napięcia/trymu, do zbadania przed `alt_kp` |
| `logs/20261017_174107.csv` | `kFlowMinSqual` (bez zmiany) | 30 | SQUAL w Active: mediana 38, p5 28, 7,8% próbek < 30; Position Hold trzymał 31 s (błąd XY 2–7 cm) i wypadł przez 0,7 s SQUAL 19–28 przy `flow_shutter` = 8191 (za ciemno). Do rozważenia: lepsze światło, potem próg ≈ 22 |
| `logs/20261017_212223.csv` | `kBrakeTau` | 0,30 → 0,45 | po puszczeniu drążka dron mija nowy punkt zadany o 5–8 cm i wraca (hamowanie ~0,45 s przy 0,2 m/s). Zmienione, wynik w następnym logu |
| `logs/20261017_214337.csv` | `kBrakeTau` (wynik) | 0,45 | cztery wychylenia (2× przód, 2× bok): przeregulowanie za nowy punkt zadany 3,2 / 5,4 / 6,5 / 3,3 cm, średnio 4,6 cm (było 5 i 8 cm); błąd XY w zawisie std. 3–5 cm, więc to poziom szumu. Zostaje 0,45 |
| `logs/20261017_212835.csv` | `alt_kp` (upstream) | 0,38 → 0,50 | zawis stoi +4…+9 cm nad zadaną i wolno wraca (w jednym locie +9,7 → +0,8 cm w 30 s); trym z napięcia jest poprawny (`thrust − thrust0` ≤ 1,2% w 3,8–3,95 V), czyli to nie napięcie, tylko za słaba zewnętrzna pętla (prędkość zadana 0,04 m/s przy błędzie 10 cm, poniżej szumu `vz` 0,05–0,09 m/s). Wynik w następnym logu |
| `logs/20261017_220222.csv` | `alt_kp` (wynik) | 0,50 | błąd wysokości schodzi do ~0 po ~12 s (było ~24 s), ale przeregulowanie po wejściu w Active +25 cm (szczyt 0,75 m; było +10…+14 cm) i drugi garb +18 cm; bateria słabsza (3,75 V przy wejściu). Niejednoznaczne, do powtórzenia na baterii ≥ 4,2 V; jeśli przeregulowanie > 20 cm, powrót do 0,38 |
| `logs/20261017_222945.csv` | `alt_kp` (wynik na pełnej baterii) | 0,50 | bateria 4,34 V, korytarz: przeregulowanie po starcie +13 cm (szczyt 0,63 m; cel < 20 cm osiągnięty), wysokość trzyma 0,50 m w całym cyklu rozładowania (gaz 0,62 → 0,77 przy napięciu 4,0 → 3,4 V), XY std. 3,5–4 cm. Błąd średni w pierwszych 20 s +5 cm (cel < 3 cm nieosiągnięty). Zostaje 0,50 |
| `logs/20261017_222945.csv` | `kBatteryWarnVolt` / `kBatteryLandVolt` (bez zmiany) | 3,50 / 3,35 V | zawis do rozładowania (~290 s lotu): spadek ok. 0,1 V na minutę, automatyczne lądowanie przy 3,34 V filtrowanego (powód `kLowBattery`), na ziemi po 2,1 s. Progi zostają |
| `logs/20261017_222945.csv` | ton ostrzeżenia o baterii (`alerts.cpp`) | 600 → 2500 Hz | ostrzeżenie przy 3,50 V nie było słyszalne w locie (piezo na 600 Hz zagłuszone przez silniki, KILL na 2500 Hz słychać). Do sprawdzenia w następnym locie |
| `logs/20261017_222945.csv` | ochrona OverG (upstream, bez zmiany) | > 2 g | zbyt mocne popchnięcie linijką wyłączyło silniki (`sensor.cpp`: przyspieszenie > 2 g → tryb PARKING). Lekkie szturchnięcia i 4 wychylenia drążkiem przeszły bez wyłączenia |
