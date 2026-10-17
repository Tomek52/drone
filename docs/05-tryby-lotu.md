# 05. Tryby lotu, przyciski, kolory LED

## Stan upstreamu (przed zmianami tego projektu)

Układ drążków **Mode 2** (domyślny w upstreamie):

| Drążek | Oś | Funkcja |
|---|---|---|
| Lewy | góra/dół | gaz (Manual) / zmiana zadanej wysokości (Auto Alt) |
| Lewy | lewo/prawo | yaw |
| Prawy | góra/dół | pitch (przód/tył) |
| Prawy | lewo/prawo | roll (lewo/prawo) |

| Przycisk | Ekran startowy | Ekran lotu |
|---|---|---|
| Lewy przedni | Stable ⇄ Sport | powrót do ekranu startowego |
| Prawy przedni | Auto Alt ⇄ Manual Alt | — |
| Ekran AtomS3 (środkowy) | przejście do ekranu lotu | start / lądowanie |
| Wciśnięcie prawego drążka | — | flip |
| Wciśnięcie lewego drążka | nieużywane | nieużywane |

| Tryb | Opis (wg przewodnika M5Stack) |
|---|---|
| Stable | samopoziomowanie, drążek zadaje kąt (limit 30°) |
| Sport | drążek zadaje prędkość kątową (akrobacje) |
| Auto Altitude | utrzymanie wysokości z ToF, lewy drążek zmienia zadaną wysokość |
| Manual Altitude | pełna kontrola gazu |

### Kolory LED drona (upstream)

| Kolor | Stan | Źródło |
|---|---|---|
| Biały | uruchamianie / autotest | DOC, UP |
| Fioletowy | kalibracja czujników — nie ruszać drona | DOC, UP |
| Przesuwające się czerwony/zielony/niebieski | gotowość (parkowanie) | DOC, UP |
| Żółty | lot, Manual Altitude, Stable | DOC, UP |
| Purpurowy (`331155` / `C71585`) | lot, Auto Altitude | DOC, UP |
| `DC669B` (różowy) | lot, Sport | UP |
| Pomarańczowy `FF9933` | flip | UP |
| Zielony | auto-lądowanie | DOC, UP |
| Czerwony | błąd odbioru od pilota | UP |
| Jasnoniebieski (cyjan) | niska bateria | DOC, UP |

## Zmiany w tym projekcie

### Wybór trybu (pilot)

| Przycisk | Ekran startowy | W locie |
|---|---|---|
| Lewy przedni | cyklicznie **Stable → Sport → Position Hold → Stable** | tylko **Position Hold → Stable**; w innych trybach jak w upstreamie (powrót do ekranu startowego + lądowanie) |
| Prawy przedni | Auto ⇄ Manual Alt; Manual przy wybranym Position Hold przełącza na Stable | — |
| **Wciśnięcie lewego drążka przez 0,5 s** | **KILL** — natychmiastowe wyłączenie silników w każdym trybie | **KILL** |

Position Hold zawsze używa Auto Altitude. Włączenie Sport lub Position Hold w powietrzu
nie jest możliwe (świadoma decyzja bezpieczeństwa — [ADR 0005](decyzje/0005-protokol-v2-i-kill-switch.md)).

### Ekran pilota

| Ekran | Element | Znaczenie |
|---|---|---|
| startowy | niebieska plakietka **POS HOLD** na obrazku trybu | wybrany Position Hold |
| startowy | pomarańczowa **POS N/A** | dron zgłasza brak czujnika flow lub kalibracji |
| startowy | czerwona **DRONE FW?** | dron z innym firmware/protokołem |
| lotu (lewy górny panel) | `STABLE` / `SPORT` (białe tło) | tryb |
| lotu | `POS HOLD` (niebieskie tło) | Position Hold |
| lotu | `STB <przyczyna>` (pomarańczowe tło) | wybrany Position Hold, ale dron leci w Stable — przyczyna wg [04-protokol-espnow.md](04-protokol-espnow.md) |
| lotu | `KILL` / `PROTO ERR` (czerwone tło) | kill switch / niezgodny protokół |

Panel lotu pokazuje tryb w miejscu, gdzie upstream wyświetlał odległość z przedniego ToF.

### Sygnalizacja na dronie (nowa)

| Sygnał | Znaczenie |
|---|---|
| LED miga czerwony/niebieski + 2 niskie piknięcia co 2 s (w parkowaniu) | pilot z innym protokołem — start zablokowany |
| jeden długi wysoki dźwięk | KILL z pilota |

Kolor LED trybu Position Hold i sygnały failsafe: gałąź `feat/position-controller`.
