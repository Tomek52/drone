# Changelog

Zmiany według gałęzi / PR (najnowsze na górze).

## `feat/phone-telemetry`

- Pilot: opcjonalny punkt dostępowy Wi-Fi i strona z podglądem telemetrii oraz zapisem CSV na telefonie
  (przytrzymanie przycisku ekranu 2 s na ekranie ustawień, domyślnie wyłączone). Zob. `13-telemetria-telefon.md`.
- `tools/gen_phone_page.py`, `tools/test_phone_page.py` (dekoder JS kontra dekoder Python w headless Chrome/Edge).

## Pierwsze testy na sprzęcie (Windows 11, 2026-10-14)

Gałęzie (każda od poprzedniej, ostatnia zawiera wszystko): `fix/windows-fastled-sd` →
`fix/joystick-telemetry-forward` → `fix/pmw3901-burst-read` → `tune/flow-calibration` →
`docs/bench-results` → `fix/flow-reject-diagnostics` → `docs/tether-results`.

- Build na Windows: `lib_ignore = SD` w obu `platformio.ini` (FastLED i `fs.h`/`FS.h`).
- Pilot: bufor TX USB-CDC 2048 B (ramki telemetrii 236 B były gubione przy 256 B).
- PMW3901: pierwszy burst po transakcji BMI270 nie dociera do czujnika; burst wykonywany dwa razy.
  Diagnostyka `-DPOSHOLD_PMW_DEBUG=1` (linie `[PMWDBG]`).
- Kalibracja flow na sprzęcie: `swap=true`, `signX=−1`, `signY=+1`, `kFlowRadPerCount = 0,00175`
  (z pochyleń względem żyroskopu), `kFlowAxesVerified = true`.
- Telemetria: pola `flow_rej_mask` (powody odrzucenia wszystkich próbek 100 Hz między ramkami)
  i `flow_rej_total`.
- Zweryfikowane na sprzęcie: protokół v2 i tryby, KILL (bez śmigieł), Position Hold „w ręku”
  (4 kierunki, fallback przy utracie flow). Pierwszy lot na uwięzi (Stable + Auto Alt): zob.
  `08-testy.md`, sekcja „Uwięź — wyniki”.

## `docs/readme-and-guides`

- Pełny główny `README.md` (etap 4): tryby, wymagania, budowanie i testy, wgrywanie,
  parowanie, M5Burner, ładowanie i bezpieczeństwo LiHV, procedura testów, strojenie,
  rozwiązywanie problemów, lista rzeczy niezweryfikowanych.
- `03-architektura.md`: diagram naszych modułów w pętli drona, katalog `common/`.
- `01-sprzet.md`: zalecenia dotyczące baterii.

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

## feat/baro-abs

- Dron: sterownik barometru BMP280 (`baro.*`), diagnostyka dolnego ToF (status, siła sygnału), monitor wiarygodności ToF
  (`height_monitor.*`, tylko obserwacja), wysokość z barometru (`abs_height.*`) i podmiana wejścia estymatora wysokości
  w trybie Stable ABS.
- Protokół: flaga `kRcFlagAbsAlt` (wysokość z barometru, trzeci stan prawego przycisku Auto/Manual/ABS) i powód `kBaroMissing` (11); pilot pokazuje `ALT ABS`, `STB ABS` / `SPT ABS`.
- Telemetria: nowe pola `baro_pa`, `baro_temp_c`, `tof_status`, `tof_sig_mcps`, `hm_trust`, `hm_pred_m`, `abs_m`;
  usunięte `ctrl_mode`, `alt_flag`, `alt_mode`, `flow_rej_total`, `acc_bias_x`, `dist_m`, `r_ref_dps` (limit ramki).
- Dron: kompensacja krawędzi przeszkód dla dolnego ToF (`tof_step.*`), wysokość bezwzględna w Auto Altitude; pole `tof_off_m`
  w telemetrii (w zamian zniknęło `loop_dt_s`).
- Stable ABS wyłączony flagą `POSHOLD_ENABLE_ABS` (domyślnie 0) po nieudanych lotach; martwa strefa obrotu w pilocie przy wysokim gazie.
