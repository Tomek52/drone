# 06. Position Hold — estymacja i sterowanie

Ten dokument opisuje, jak dron wie, gdzie jest (estymacja), jak utrzymuje
pozycję (sterowanie) i co robi, gdy coś pójdzie nie tak (nadzorca i failsafe'y).
Wszystkie parametry: `drone/lib/poshold/src/poshold_config.hpp`.

## Układy współrzędnych

| Układ | Osie | Uwagi |
|---|---|---|
| **body** (ciało drona) | x — do przodu, y — w prawo, z — w dół | konwencja upstreamu (`sensor.cpp`) |
| **heading** (kursowy) | jak body, ale bez przechylenia i pochylenia | płaszczyzna pozioma, obrócona razem z dronem wokół osi pionowej |
| **world** (świat) | heading obrócony o kąt yaw | początek = miejsce startu / ostatniej zmiany trybu |

Kąty: roll dodatni = prawa strona w dół, pitch dodatni = nos w górę,
yaw dodatni = obrót w prawo (patrząc z góry). Prędkości kątowe p, q, r — wokół osi x, y, z.

⚠️ Kąt yaw pochodzi z filtru Madgwicka bez magnetometru i **dryfuje** (rzędu stopni
na minutę). Wpływa to tylko na kierunek, w którym całkowana jest pozycja
w układzie world — w krótkich lotach jest to akceptowalne.

## Estymacja prędkości i pozycji

```mermaid
flowchart LR
    IMU["BMI270<br/>400 Hz"] --> MAD["Madgwick (upstream)<br/>roll, pitch, yaw"]
    IMU --> PRED["predykcja:<br/>v += (a_h − b)·dt"]
    MAD --> PRED
    PMW["PMW3901<br/>100 Hz"] --> FLOW["prędkość z flow<br/>v_flow = h·(ω − ω_gyro)"]
    TOF["ToF dolny<br/>~30 Hz → Kalman (upstream)"] --> H["h = d·cos(roll)·cos(pitch)"]
    H --> FLOW
    MAD --> H
    IMU -->|"średnie p, q"| FLOW
    FLOW --> GATE{"bramki jakości"}
    GATE -->|"OK"| CORR["korekta:<br/>v += k·(v_flow − v)<br/>b −= k_b·(v_flow − v)"]
    PRED --> CORR
    CORR --> POS["pozycja world:<br/>p += R(yaw)·v·dt"]
```

Kod: `drone/lib/poshold/src/position_estimator.*` (niezależny od sprzętu, testowany
`native`), sterownik czujnika `drone/src/pmw3901.*`, połączenie z pętlą upstreamu
`drone/src/poshold_glue.*`.

### 1. Wysokość nad podłożem

Dolny ToF mierzy odległość wzdłuż swojej osi (skośnie, gdy dron jest przechylony).
Upstream filtruje ją filtrem Kalmana (`Altitude2`). Wysokość prostopadła
(płaska podłoga):

```
h = d · cos(roll) · cos(pitch)
```

### 2. Prędkość z optical flow

PMW3901 zwraca przyrosty `Δx, Δy` (zliczenia) od poprzedniego odczytu. Po
przemapowaniu osi czujnika na osie drona (`kFlowSwapXY`, `kFlowSignX/Y`) prędkość
kątowa „przesuwania się podłogi”:

```
ωx = Δx_body · K / Δt        ωy = Δy_body · K / Δt        K = kFlowRadPerCount [rad/zliczenie]
```

Ruch obrazu ma dwie przyczyny: przesunięcie drona i jego obrót. Gdy dron pochyla
nos w górę (q > 0), oś czujnika „przesuwa się” po podłodze do przodu — tak samo
jak przy locie do przodu. Przy przechyle w prawo (p > 0) — w lewo. Stąd:

```
ωx = vx / h + q      →   vx = h · (ωx − q)
ωy = vy / h − p      →   vy = h · (ωy + p)
```

p i q są uśredniane ze wszystkich próbek żyroskopu (400 Hz) z okresu między
odczytami flow.

### 3. Fuzja z akcelerometrem (filtr komplementarny)

Siła właściwa z akcelerometru w układzie body `f` [g] jest obracana do układu
heading (usuwa to wpływ grawitacji):

```
a_x = g · ( cos(pitch)·fx + sin(pitch)·sin(roll)·fy + sin(pitch)·cos(roll)·fz )
a_y = g · ( cos(roll)·fy − sin(roll)·fz )
```

Predykcja co 2,5 ms: `v += (a − b)·dt`. Korekta przy każdej dobrej próbce flow
(co 10 ms), z błędem `e = v_flow − v`:

```
v += k·e,   k = Δt / (τ_v + Δt),          τ_v = kVelCorrectionTau
b −= k_b·e, k_b = Δt / (τ_b · τ_v),       τ_b = kAccelBiasTau
```

Akcelerometr wygładza zaszumioną i skwantowaną prędkość z flow; flow usuwa
dryf całkowania akcelerometru i estymuje jego bias. Uzasadnienie wyboru:
[ADR 0004](decyzje/0004-filtr-komplementarny.md).

### 4. Pozycja

```
p_world += R(yaw) · v · Δt
```

Pozycja jest całką prędkości — **dryfuje** (błąd skali flow, kwantyzacja, dryf yaw).
To akceptowane założenie tej wersji: utrzymanie pozycji ma działać przez
czas jednego lotu (~4 min), z dokładnością rzędu decymetrów.

### 5. Bramki jakości optical flow

Próbka jest odrzucana (`flow_reject` w telemetrii), gdy:

| Kod | Warunek | Parametr |
|---|---|---|
| 1 | brak odczytu czujnika lub brak pomiaru ToF | — |
| 2 | wysokość < 0,10 m | `kFlowMinHeight` |
| 3 | wysokość > 1,50 m | `kFlowMaxHeight` |
| 4 | SQUAL < 30 (słaba tekstura / oświetlenie) | `kFlowMinSqual` |
| 5 | \|p\| lub \|q\| > 3 rad/s | `kFlowMaxBodyRate` |
| 6 | \|roll\| lub \|pitch\| > 0,45 rad (~26°) | `kFlowMaxTilt` |

Histereza: flow jest „ważny” (`flow_valid`) po 5 kolejnych dobrych próbkach
(50 ms), „nieważny” po 3 kolejnych złych (30 ms). Bez korekt prędkość jest dalej
całkowana z akcelerometru, ale tryb Position Hold po utracie ważności flow
przełączy się na Stable + Auto Altitude (gałąź `feat/position-controller`).

### Rozdzielczość i ograniczenia

- Przy domyślnym `K ≈ 0,021 rad` jedno zliczenie na próbkę (10 ms) na wysokości
  0,5 m odpowiada ~1 m/s. Pojedyncze próbki są więc mocno skwantowane — dlatego
  korekta jest łagodna (τ_v = 0,15 s), a prędkość wynikowa ma szum rzędu
  ±0,05 m/s (sprawdzone w symulacji w testach `native`). ⚠️ Rzeczywista
  rozdzielczość czujnika (zliczenia na piksel) do weryfikacji kalibracją.
- Założenie płaskiej podłogi: progi, stopnie i ruchome obiekty pod dronem
  zaburzają pomiar.
- Wysokość to odległość od powierzchni pod dronem (dron „śledzi teren” nad przeszkodami); wysokość
  bezwzględna jest zaplanowana: [10-plany.md](10-plany.md).
- Orientacja montażu PMW3901 na StampFly nie jest udokumentowana — mapowanie osi
  trzeba wyznaczyć próbą z przenoszeniem drona (zob. niżej). Do tego czasu
  `kFlowAxesVerified = false` i Position Hold nie da się włączyć.

## Kalibracja optical flow

Szczegółowa procedura: [08-testy.md](08-testy.md#kalibracja-optical-flow-przenoszenie-w-ręku).
Skrypt `tools/flow_calibration.py` wylicza z logu `kFlowSwapXY`, `kFlowSignX`,
`kFlowSignY` i `kFlowRadPerCount`.

## Sterowanie

Kod: `drone/lib/poshold/src/position_controller.*` (testowany `native`, w tym
symulacja w pętli zamkniętej), połączenie z upstreamem: `drone/src/poshold_glue.*`.

```mermaid
flowchart LR
    ST["drążki"] --> SP["punkt zadany XY (world)<br/>+ wysokość zadana"]
    SP --> PE["błąd pozycji<br/>→ układ heading"]
    EST["estymator: p, v"] --> PE
    PE -->|"× kPosKp"| VS["prędkość zadana<br/>(+ feed-forward z drążka)<br/>limit kMaxSpeed"]
    VS --> VE["błąd prędkości"]
    EST --> VE
    VE -->|"PI: kVelKp, kVelKi"| ACC["przyspieszenie zadane<br/>limit g·tan(10°)"]
    ACC -->|"pitch = −atan(ax/g)<br/>roll = atan(ay/g)"| ANG["kąty zadane ±10°"]
    ANG --> UPS["regulator kąta i prędkości kątowej<br/>(upstream)"]
    SP -->|"Alt_ref"| ALT["regulator wysokości Auto Alt<br/>(upstream)"]
```

### Drążki w Position Hold (Mode 2)

| Drążek | Ruch | Tryb ciągły (domyślny) | Tryb krokowy (`kStepMode = true`) |
|---|---|---|---|
| prawy | przód/tył | punkt zadany przesuwa się z prędkością ∝ wychyleniu (pełne = `kMaxSpeed` 0,5 m/s) | każde wychylenie > 60% = jeden krok `kStepXY` (30 cm) |
| prawy | lewo/prawo | jw. | jw. |
| lewy | góra/dół | wysokość zadana zmienia się z prędkością ∝ wychyleniu (pełne = 0,3 m/s) | krok `kStepZ` (20 cm) |
| lewy | lewo/prawo | obrót, pełne wychylenie = 90°/s | jw. (ciągły) |

Kierunki „przód/prawo” są w **układzie drona** (zgodnie z wymaganiem). Strefa martwa
drążków: 10%. Wysokość zadana: 0,20–1,20 m (w oknie wiarygodności optical flow).

Po puszczeniu drążka punkt zadany jest ustawiany w przewidywanym miejscu zatrzymania
(`pozycja + prędkość · kBrakeTau`) — dron hamuje i zostaje tam, zamiast cofać się do
punktu, który już minął. Podczas lotu z drążkiem punkt zadany nie wyprzedza drona
o więcej niż `kMaxLead` (0,5 m).

### Wzory

```
e_h   = R(−yaw) · (sp − p)                          błąd pozycji w układzie heading
v_sp  = kPosKp · e_h + v_ff,         |v_sp| ≤ kMaxSpeed
a     = kVelKp · (v_sp − v) + I,     I += kVelKi·(v_sp − v)·dt, |I| ≤ kVelIMax
|a|   ≤ g · tan(kMaxTilt)
pitch_ref = −atan(a_x / g)          (nos w dół = przyspieszenie do przodu)
roll_ref  =  atan(a_y / g)          (prawa strona w dół = w prawo)
```

Referencje kątów zastępują w `angle_control()` upstreamu wartości wyliczone z drążków
(haczyk `poshold_override_attitude`), a dalej działa niezmieniony regulator kąta
i prędkości kątowej. Wysokość zadana trafia do `Alt_ref` istniejącego regulatora
Auto Altitude. W trybie Position Hold flip jest zablokowany.

## Nadzorca i bezpieczeństwo

Kod: `drone/lib/poshold/src/poshold_supervisor.*` (testowany `native`).
Decyzje: [ADR 0006](decyzje/0006-nadzorca-i-failsafe.md).

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> WaitEngage: Position Hold wybrany, lot, Auto Alt
    WaitEngage --> Active: po starcie, flow OK, wysokość OK
    WaitEngage --> Fallback: brak czujnika/kalibracji lub 3 s bez warunków
    Active --> Fallback: flow lub wysokość złe przez 0,2 s, geofence
    Active --> Idle: pilot wybrał Stable lub lądowanie poniżej 0,10 m
    Fallback --> Idle: po wylądowaniu
```

**Fallback** = Stable + Auto Altitude sterowane drążkami (drążki znów zadają kąty!).
Nie ma automatycznego powrotu do Position Hold w powietrzu — Position Hold można
ponownie wybrać dopiero po wylądowaniu.

| Sytuacja | Reakcja | Sygnał |
|---|---|---|
| Złe optical flow / wysokość poza oknem > 0,2 s | fallback do Stable + Auto Alt | 3 krótkie piknięcia, LED miga na niebiesko, pilot: `STB FLOW` / `STB HEIGHT` |
| Brak czujnika PMW3901 lub brak kalibracji | Position Hold nie wchodzi, lot w Stable | jw.; na ziemi pilot: `POS N/A` |
| Utrata łączności (> 0,1 s), Auto Alt | **zawis** z neutralnymi drążkami (Position Hold dalej trzyma XY, jeśli aktywny) przez 3 s, potem łagodne auto-lądowanie upstreamu (XY trzymane do 0,10 m) | szybkie piknięcia, pilot: `STB LINK` |
| Utrata łączności, Manual Alt | natychmiastowe auto-lądowanie (jak w upstreamie) | jw. |
| Bateria < 3,50 V (filtrowana) | ostrzeżenie | długie niskie dźwięki |
| Bateria < 3,35 V przez 2 s | **auto-lądowanie** (w każdym trybie) | pilot: `STB BATT` |
| Odległość od startu > 2,5 m (`kFenceRadius`·`kFenceLandFactor`) | fallback + **auto-lądowanie** | pilot: `STB FENCE` |
| Punkt zadany | nigdy dalej niż 2,0 m od startu | — |
| Kąty w Position Hold | twardy limit ±10° (plus ±30° upstreamu) | — |
| KILL z pilota (lewy drążek 0,5 s) | natychmiastowe zatrzymanie silników, każdy tryb | długi dźwięk, pilot: `KILL` |

⚠️ Progi baterii są wstępne — napięcie spada pod obciążeniem. Ustal je z logów
(`plot_log.py --groups battery`), zob. [07-strojenie.md](07-strojenie.md).

Punktem startu (początek układu world, środek geofence) jest miejsce, w którym dron
przeszedł z parkowania do lotu.
