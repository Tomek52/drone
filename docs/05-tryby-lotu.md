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

Nowy tryb **Position Hold** i jego kolor zostaną opisane tutaj w gałęziach
`feat/joystick-mode` i `feat/position-controller`.
