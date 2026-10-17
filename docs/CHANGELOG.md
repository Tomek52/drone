# Changelog

Zmiany według gałęzi / PR (najnowsze na górze).

## `feat/position-controller`

- Regulator Position Hold (`drone/lib/poshold/src/position_controller.*`): pozycja (P) →
  prędkość (PI z anti wind-up) → kąty (limit 10°) → regulator kąta upstreamu; wysokość przez
  `Alt_ref` istniejącego Auto Alt; drążki w układzie drona (ciągły: 0,5 m/s, krokowy: 30 cm / 20 cm),
  hamowanie w przewidywanym punkcie zatrzymania, limit wyprzedzenia, geofence punktu zadanego.
- Nadzorca (`drone/lib/poshold/src/poshold_supervisor.*`, ADR 0006): wejście po auto-starcie,
  fallback do Stable + Auto Alt przy złym flow/wysokości (zatrzaskiwany), zawis 3 s po utracie
  łączności i łagodne lądowanie, lądowanie przy niskiej baterii i po przekroczeniu geofence.
- Haczyki w upstreamie: nadzorca przed maszyną stanów, `poshold_after_get_command()`,
  `poshold_override_attitude()` w `angle_control()`, warunek lądowania po utracie łączności.
- LED: stały niebieski = Position Hold, migający niebieski = fallback; sygnały dźwiękowe;
  status dla pilota z faktycznym trybem i przyczyną.
- Nieblokujące komunikaty tekstowe na USB w locie (`telemetry_text`).
- Telemetria: stan i przyczyna, punkt i prędkość zadana, odległość od startu, napięcie filtrowane.
- Testy `native`: 15 testów regulatora (w tym symulacja w pętli zamkniętej) i 14 testów nadzorcy.
- Dokumentacja: sterowanie i nadzorca w `06-position-hold.md`, strojenie krok po kroku,
  procedury: test „w ręku”, uwięź, niski lot; rozwiązywanie problemów.

## `feat/joystick-mode`

- Protokół ESP-NOW v2 (ADR 0005, `common/stampfly_link/src/rc_protocol.hpp`): pole wersji,
  tryb lotu, flaga kill; pakiet statusu dron → pilot (5 Hz) z faktycznym trybem i przyczyną.
- Dron: odrzuca pakiety innej wersji (start niemożliwy), sygnalizuje niezgodność LED-em i buzzerem;
  kill switch zatrzymuje silniki w każdym trybie lotu (`drone/src/rc_link.*`).
- Dron: nieblokujący odtwarzacz sygnałów buzzera i nadpisywanie LED (`drone/src/alerts.*`).
- Pilot: cykl trybów Stable → Sport → Position Hold na lewym przycisku (na ziemi), w locie
  tylko Position Hold → Stable; Position Hold wymusza Auto Alt; kill switch = lewy drążek 0,5 s;
  oznaczenia trybu, fallbacku i błędów na ekranie (`joystick/src/flight_mode.*`).
- Logika pilota w `common/stampfly_link/src/joystick_logic.hpp` z testami `native` (12 przypadków).
- Telemetria: `rc_proto_ok`, `rc_flight_mode`, `rc_kill`.
- Position Hold w tej gałęzi jest latany jako Stable + Auto Alt (regulator w następnej gałęzi).

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
