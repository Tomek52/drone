# Changelog

Zmiany według gałęzi / PR (najnowsze na górze).

## `feat/position-estimator`

- Sterownik PMW3901MB (`drone/src/pmw3901.*`): drugie urządzenie na SPI2 (CS G12),
  kontrola Product ID, sekwencja inicjalizacji wg sterownika Bitcraze (MIT), odczyt burst z SQUAL.
- Estymator prędkości i pozycji (`drone/lib/poshold/src/position_estimator.*`, ADR 0004):
  prędkość z flow skalowana wysokością z ToF (z kompensacją przechyłu), kompensacja obrotu
  z żyroskopu, filtr komplementarny z akcelerometrem i estymacją biasu, pozycja XY przez całkowanie,
  bramki jakości (SQUAL, okno wysokości, prędkość obrotu, przechył) z histerezą.
- Jeden plik konfiguracji `drone/lib/poshold/src/poshold_config.hpp`.
- Haczyki w upstreamie: `poshold_init()` po `sensor_init()`, `poshold_update_estimation()` po `sensor_read()`.
- Telemetria: 19 nowych pól (flow, prędkości, pozycja, wysokość, bias, skumulowane zliczenia).
- `tools/flow_calibration.py`: wyznaczanie mapowania osi i skali flow z próby przenoszenia.
- Testy `native` estymatora (15 przypadków z symulatorem ruchu) i test skryptu kalibracji.
- Dokumentacja: `06-position-hold.md` (estymacja), procedury kalibracji w `08-testy.md`,
  tabela problemów z optical flow w `09-rozwiazywanie-problemow.md`.
- **Bez zmian w sterowaniu** — estymator działa tylko na potrzeby telemetrii.

## `feat/telemetry`

- Binarna telemetria do strojenia (ADR 0003): ramki `A5 5A` z sumą Fletcher-16,
  schemat pól wysyłany co 2 s w fragmentach.
- Dron: USB 50 Hz + ESP-NOW 25 Hz do pilota, zapis nieblokujący (`drone/src/telemetry_stream.*`,
  haczyk w `loop_400Hz()`).
- Pilot: przekazywanie ramek telemetrii z ESP-NOW na USB (`joystick/src/telemetry_forward.*`,
  haczyk w `OnDataRecv()`).
- Wspólna biblioteka `common/stampfly_link` (kodek ramek) dla drona, pilota i testów.
- `tools/telemetry_logger.py` (zapis CSV, opcjonalnie surowy `.bin`), `tools/plot_log.py`
  (wykresy do strojenia), dekoder `tools/stampfly_telemetry.py` z testami.
- CI: testy narzędzi Pythona.
- Dokumentacja: protokół telemetrii, procedura testu telemetrii, podstawy strojenia.

## `chore/repo-setup`

- Przypięte wersje platformy i bibliotek PlatformIO w obu projektach (ADR 0002).
- LVGL 8.3.10 dla pilota pobierany z rejestru PlatformIO zamiast archiwum ZIP z GitHuba.
- Środowisko `native` z Unity i pierwszym testem (`test_math_utils`), biblioteka `drone/lib/poshold`.
- GitHub Actions: kompilacja drona i pilota, testy `native`.
- Szkielet dokumentacji `docs/`, ADR 0001 (struktura repozytorium), ADR 0002 (toolchain).
- Plik `stampfly.code-workspace` dla VS Code, główny `README.md`, `.gitignore`.

## `main` (import)

- Import `m5stack/M5StampFly@fa7b6a5` do `drone/` (git subtree, pełna historia).
- Import `m5stack/Atom-JoyStick@e9f53fb` (`examples/StampFlyController`) do `joystick/`.
- Bez zmian w kodzie upstreamu.
