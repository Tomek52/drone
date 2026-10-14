# 04. Protokół ESP-NOW

Komunikacja pilot ⇄ dron odbywa się przez ESP-NOW na kanale Wi-Fi wybranym przy
parowaniu (domyślnie kanał 1). Wszystkie wartości wielobajtowe są little-endian
(natywny format ESP32-S3).

## Wersja 1 (upstream, bez pola wersji)

### Parowanie

Dron po starcie przez ok. 2,5 s (50 × co 50 ms) rozgłasza na adres `FF:FF:FF:FF:FF:FF`
pakiet 11 B:

| Bajt | Zawartość |
|---|---|
| 0 | kanał Wi-Fi drona |
| 1–6 | adres MAC drona |
| 7–10 | znacznik `AA 55 16 88` |

Pilot w trybie parowania przełącza kanały 1–14, a po odebraniu znacznika zapisuje
kanał i MAC drona w SPIFFS (`/peer_info.txt`).

### Pilot → dron (sterowanie), 25 B, 100 Hz

| Bajt | Typ | Zawartość |
|---|---|---|
| 0–2 | u8×3 | 3 młodsze bajty MAC drona (filtr przeciw obcym pilotom) |
| 3–6 | float | rudder (yaw), −1…1 |
| 7–10 | float | throttle, −1…1 |
| 11–14 | float | aileron (roll), −1…1 |
| 15–18 | float | elevator (pitch), −1…1 |
| 19 | u8 | impuls ARM / start-lądowanie |
| 20 | u8 | flip (wciśnięty prawy drążek) |
| 21 | u8 | tryb sterowania: 0 = Stable (ANGLE), 1 = Sport (RATE) |
| 22 | u8 | tryb wysokości: 4 = Auto, 5 = Manual |
| 23 | u8 | żądanie resetu AHRS |
| 24 | u8 | suma kontrolna = suma bajtów 0–23 (mod 256) |

Dron nie sprawdza długości pakietu. Pierwszy nadawca, którego pakiet dotrze do drona,
zostaje zapisany jako odbiorca telemetrii.

### Dron → pilot (telemetria), 120 B, 40 Hz

- Nagłówek `99 99` — jednorazowo po starcie: wzmocnienia regulatorów (float).
- Nagłówek `88 88` — 28 pól `float` (czas, kąty, prędkości kątowe, wartości zadane,
  napięcie, przyspieszenia, wysokości) + `Alt_flag` (u8), `Mode` (u8), `RangeFront` (u16).
  Szczegółowa lista pól: `drone/src/telemetry.cpp`, funkcja `make_telemetry_data()`.

### Failsafe w wersji 1

Dron liczy obiegi pętli od ostatniego poprawnego pakietu; po 40 obiegach (100 ms)
przechodzi w auto-lądowanie.

## Wersja 2

Wprowadzana w gałęzi `feat/joystick-mode` (pole wersji protokołu, tryb lotu,
flaga natychmiastowego wyłączenia silników). Opis zostanie dodany w tym PR.
