# ADR 0002: Przypięcie wersji platformy i bibliotek PlatformIO

- Status: przyjęta
- Data: 2026-10-14
- Gałąź: `chore/repo-setup`

## Kontekst

- `drone/platformio.ini` w upstreamie nie przypinał wersji (`platform = espressif32`,
  `fastled/FastLED` bez wersji). Kod drona używa API Arduino-ESP32 **2.x**
  (`ledcSetup`, `ledcAttachPin`, `timerBegin(0, 80, true)`), które w 3.x nie istnieje.
  Nowe wydanie platformy lub biblioteki mogłoby z dnia na dzień zepsuć kompilację.
- `joystick/platformio.ini` pobierał LVGL jako archiwum ZIP z GitHuba
  (`https://github.com/lvgl/lvgl/archive/refs/tags/v8.3.10.zip`). Takie archiwa są
  generowane dynamicznie i bywają blokowane przez proxy (w środowisku chmurowym
  zwracały 403), przez co kompilacja pilota była niemożliwa.
- Globalna sekcja `[env]` w `drone/platformio.ini` uniemożliwiała dodanie środowiska
  `native` (dziedziczyłoby platformę ESP32 i biblioteki Arduino).

## Decyzja

- Dron: `espressif32 @ 6.12.0` (Arduino-ESP32 2.0.17), `FastLED @ 3.10.7`,
  `INA3221 @ 0.0.1`, `OneButton @ 2.6.2` — dokładnie te wersje, które upstream
  rozwiązywał w dniu importu (rozmiar firmware'u identyczny: 1 089 625 B flash).
- Sekcja `[env]` drona przemianowana na `[esp32_common]`, środowisko firmware'u
  używa `extends = esp32_common`; dodane `[env:native]` i `default_envs`.
- Pilot: `M5GFX @ 0.2.33`, `FastLED @ 3.10.7`, `lvgl/lvgl @ 8.3.10` z rejestru
  PlatformIO (ta sama wersja LVGL co w upstreamie).

## Rozważane alternatywy

| Alternatywa | Dlaczego nie |
|---|---|
| Zostawić bez przypięcia | Niepowtarzalne buildy, ryzyko przejścia na Arduino 3.x. |
| `pioarduino` (Arduino 3.x) | Wymaga przepisania PWM i timerów w kodzie upstreamu — sprzeczne z zasadą minimalnych zmian. |
| LVGL przez URL repozytorium git | Działa, ale rejestr PlatformIO jest prostszy i szybszy. |

## Konsekwencje

- Aktualizacje wersji robimy świadomie, w osobnym PR, z ponowną kompilacją i testami.
- Przy `git subtree pull` z upstreamu możliwy konflikt w `platformio.ini` (zmiany
  oznaczone `[POSHOLD]`).

## Korekta po testach na sprzęcie (2026-10-17)

- **Dron: `FastLED @ 3.7.8` zamiast 3.10.7.** Z 3.10.7 wszystkie trzy diody drona (moduł StampS3
  oraz dwie diody płytki) świecą stale na biało i nie reagują na dane z firmware'u, nawet przy
  wymuszeniu czerwonego/zielonego/niebieskiego. Z 3.7.8 te same kolory działają (moduł czerwony,
  górna dioda płytki zielona, dolna niebieska). Firmware jest też o ~280 KB mniejszy.
- Pilot nadal używa 3.10.7 (jego diody nie były sprawdzane); do ujednolicenia po osobnym teście.
- `lib_ignore = SD` (Windows) pozostaje dla pilota z 3.10.7; dla drona z 3.7.8 nie był już
  weryfikowany osobno.
