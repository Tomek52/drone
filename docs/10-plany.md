# 10. Plany i otwarte sprawy

Rzeczy zaplanowane na później. Nie są częścią obecnego firmware'u.

## Wysokość bezwzględna przy przelatywaniu nad przeszkodami

**Cel.** Dron ma utrzymywać wysokość względem miejsca startu (podłogi), a nie względem tego, co
akurat jest pod nim. Przykład: start z podłogi, przelot nad przeszkodą (łóżko, stół, pudło) bez
zmiany wysokości, i odwrotnie: start z przeszkody i zjazd na podłogę bez opadania.

**Jak jest dziś.** Wysokość w Position Hold to odległość od powierzchni pod dronem (AGL): dolny ToF
VL53L3C, filtr Kalmana upstreamu `Altitude2`, regulator `alt_err = Alt_ref − Altitude2`. Dron
„śledzi teren": nad przeszkodą wznosi się o jej wysokość, po zjeździe opada. To bezpieczne
zachowanie domyślne i zostaje jako tryb podstawowy.

**Pomysł.**
- Rozdzielić dwie wielkości: AGL (skala optical flow, lądowanie, wykrywanie ziemi) i wysokość
  bezwzględna (regulator wysokości w nowym trybie).
- Rozpoznawanie zmiany terenu: dron nie zmieni wysokości o kilkanaście cm w 100 ms, więc skok ToF,
  którego nie potwierdza przyspieszenie pionowe z IMU, oznacza zmianę terenu. Estymator jedzie wtedy
  na akcelerometrze (krótko) i przesuwa przesunięcie terenu, a `Alt_ref` w jednostkach AGL jest
  korygowany o skok.
- Barometr BMP280 jest na pokładzie i nieużywany, ale w pomieszczeniu jest zaszumiony, dryfuje,
  a strumień powietrza ze śmigieł zaburza pomiar. Raczej tylko jako uzupełnienie.

**Ograniczenia i ryzyka.**
- Optical flow działa tylko w oknie 0,10–1,50 m nad powierzchnią pod dronem (`kFlowMinHeight`,
  `kFlowMaxHeight`); wysokość bezwzględna musi mieścić się w oknie dla każdej przeszkody na trasie.
- Lądowanie i wykrywanie ziemi muszą zostać na AGL.
- Zmiana dotyka pętli wysokości w kodzie upstreamu (odrzucanie wyników ToF w filtrze, przesuwanie
  `Alt_ref`), czyli wymaga zgody na odstępstwo od zasady „w upstreamie tylko haczyki `[POSHOLD]`".
- Błędne rozpoznanie schodka (szum ToF, krawędź mebla, częściowe zasłonięcie wiązki) wynosi drona
  wyżej lub niżej niż trzeba, więc potrzebne są progi, testy `native` i test na uwięzi.

**Pierwszy krok (bez ryzyka).** Zebrać dane: dron w ręku, silniki wyłączone, stała wysokość,
przesuwanie nad przeszkodami o różnej wysokości i krawędziach (pudło, krawędź stołu, schody).
Sprawdzić w logu (`tof_raw_mm`, `alt_est_m`, `vz_mps`), jak ostry jest schodek i czy da się go
odróżnić od ruchu pionowego.

## Inne otwarte sprawy po pierwszej kampanii testów

| Sprawa | Stan |
|---|---|
| Utrata łączności (zawis ~3 s i lądowanie) z zapisanym logiem | log 2026-10-17 (`logs/20261017_230731.csv`, symulacja przez pilota na gałęzi `test/link-cut`) pokazał błąd: lądowanie startowało po 0,1 s zamiast po 3 s zawisu (upstream odcina łączność przy 40 pętlach, a nadzorca zezwalał na lądowanie dopiero przy `link_age` > 0,1 s). Poprawka w `poshold_supervisor.cpp` na gałęzi `fix/link-loss-hover`; powtórzony test (`logs/20261017_231554.csv`): łączność utracona po 15 s Position Hold, `ph_reason` = 7, zawis 3,4 s, lądowanie 1,1 s, silniki off przy 0,1 m. Podczas zawisu wysokość wahała się 0,54–0,78 m (cel 0,5 m, odchylenie std. 0,08 m wobec 0,03 m wcześniej), przyczyna niewyjaśniona (podobne wahania są przy napięciu < 3,7 V, ale wychylenie zaczęło się dokładnie przy utracie łączności) |
| Progi baterii z logów (`kBatteryWarnVolt`, `kBatteryLandVolt`) | przeanalizowane 16 lotów (4,32 → 3,54 V pod obciążeniem): w zawisie napięcie spada o 0,3–0,4 V względem spoczynkowego, najniższe filtrowane 3,56 V, progu 3,50 V żaden lot nie osiągnął. Progi zostają bez zmian; do ich weryfikacji potrzebny jest lot na uwięzi do ostrzeżenia |
| Strojenie `kPosKp`, `kVelKp`, `kVelKi` z odpowiedzi na wychylenie drążka | zrobione w locie 2026-10-17 (popchnięcia i drążki), parametry zostają |
| Wysokość przy baterii < 3,6 V: wahania ±0,15 m (okres ~4,5 s) | czujnik ToF wykluczony (szum 5 → 6 mm, brak zer i skoków > 100 mm, `alt_tof` = `alt_est`), wzmocnienie ciągu w przybliżeniu stałe (12–16 m/s² na 1,0 gazu), sprzężenie napięcia wykluczone; przyczyna nieznana, kolejny krok to eksperyment (np. niższe `z_dot_kp`/`alt_kp` przy niskim napięciu) albo przyjęcie granicy ~3,6 V |
| Dryf przed wejściem w Active (~15 cm w pierwszych 3–4 s) i trym przechylenia | do zbadania |
| Pion: błąd ustalony +4…+9 cm i wolny powrót po przeregulowaniu przy starcie | `alt_kp` 0,38 → 0,50 do zweryfikowania w locie; hipoteza o wpływie napięcia na trym obalona (trym trafia w 1,2%) |
| Pilot: FastLED 3.10.7 (drona przestawiono na 3.7.8) | diody pilota niesprawdzone |
| Dystans z przenoszenia 1 m ±15% | skala potwierdzona żyroskopem, dystansu nie da się zweryfikować ręcznie |
| Magnetometr BMM150 do `yaw` | `yaw` dryfuje ±15° w 20 s; pozycja nie jest od tego zależna |
| `thrust_duty` zamrożone po KILL w telemetrii, `alt_mode` na ziemi | drobne |
| Test niskiej baterii 2026-10-17 (`logs/20261017_232225.csv`, start 3,97 V) | dron wisiał niestabilnie (wysokość std. 0,07–0,17 m, XY do 0,13 m), dwa razy stracił wysokość przy 3,40 i 3,36 V filtrowanego (gaz 0,80–0,89) i spadł (powód 5 i 4); próg lądowania 3,35 V jest za niski dla słabszej baterii. Progi zostają na decyzję właściciela (3,50 / 3,35 V); ostrzeżenie z drona (buzzer) nie było słyszalne mimo ~5 min poniżej 3,50 V, dlatego dodano ostrzeżenie w pilocie (`feat/pilot-low-battery-alert`) |
| Wysokość bezwzględna z barometru (tryb Stable ABS) i odporność na oślepienie ToF | po zdarzeniu w parku (dron wzniósł się na 20–30 m po oślepieniu ToF w Position Hold i Stable+AutoAlt, KILL) w toku: etap 1 — telemetria BMP280 i statusu ToF (`feat/baro-abs`), pomiar w ręku w mieszkaniu (stół, łóżko, sufit ok. 2,4 m); etap 2 — fuzja barometr + akcelerometr + ToF i wykrycie złego ToF (kontrolowane lądowanie, tylko w trybach ze ToF); etap 3 — tryb Stable ABS w pilocie. Ryzyko: upstream trzyma ostatni odczyt `Range` przy `RawRange` ≤ 20 (`sensor.cpp`), więc oślepiony czujnik nie daje alarmu. Pomiary w ręku 2026-10-19 (logi `20261019_19*.csv`): BMP280 działa, szum próbki 1,4–2,2 Pa (12–17 cm), uśrednianie ok. 1 s daje 3–4 cm; przy stałej temperaturze płytki (ok. 40 °C) podłoga 0, łóżko 53 cm → 61 cm, szafa 248 cm → 247 cm (11,0–11,5 Pa/m); dryf temperaturowy ok. 1–1,4 Pa/K (płytka nagrzewa się z 27 do 40 °C w kilkanaście minut), więc barometr nadaje się do zmian względnych, nie do wysokości absolutnej bez korekty temperaturowej. Latarka (LED, noc) na dolny ToF: odczyt z 28 mm skacze do 0–691 mm, `tof_status` 7 i 14, `tof_sig_mcps` do 292 (normalnie 36); w części próbek status 0 i fałszywe 50–67 mm, więc samego statusu nie wystarczy do wykrycia oślepienia. Lot kontrolny 2026-10-19 (`20261019_194111.csv`, 2 starty): podmuch ze śmigieł obniża ciśnienie na płytce o 5–6,5 Pa przy gazie 0,65–0,70 (pozorny wzrost wysokości +0,4…+0,55 m, rozrzut 0,3–0,5 m), więc barometr w locie nie nadaje się do porównań z dokładnością ±25 cm; monitor (`height_monitor`, tylko obserwacja) dostał szeroką bramkę 1,5 m / 1,5 s i stałą czasową offsetu 10 s, co nadal wychwytuje wznoszenie o metry. Na pierwszym starcie estymator pozycji uciekł w czasie rozruchu na ziemi (x −2 m, y +3,5 m przy wysokości 0,03–0,15 m), geofence zakończył start. Lot schodkowy (`20261019_200020.csv`, 0,5 → 1,2 m, bateria 3,6 → 3,5 V): przesunięcie ciśnienia od podmuchu jest prawie stałe, 6–9 Pa, w całym zakresie 0,5–1,3 m przy gazie 0,72–0,75 (nie zależy wyraźnie od wysokości), monitor uczy się go w ok. 20 s i błąd `hm_pred_m − alt` spada z +0,3 m do ±0,1 m, `hm_trust` = 1 przez cały lot. Kompensacja z gazu niepotrzebna; zostaje uczenie offsetu w locie. W Position Hold punkt zadany wysokości jest ograniczony do 1,20 m (`kMaxAltitude`) i rośnie z prędkością 0,3 m/s przy pełnym drążku, dlatego dron nie wznosił się wyżej mimo pełnego gazu. Rozrzut wysokości przy 3,5–3,6 V: 0,13–0,17 m. Logger zakończył się w trakcie lotu (100 s), a dron wylądował po utracie łączności z pilotem; hipoteza: zamknięcie portu COM resetuje pilota (USB-CDC, DTR/RTS), więc nagrywać dłużej niż lot |
| Stable ABS: zaimplementowany (`feat/baro-abs`, [11-stable-abs.md](11-stable-abs.md)), pierwszy lot 2026-10-19 nieudany (falowanie ±0,5 m, zła wysokość na rampie startowej), eksperymentalny | do przetestowania w mieszkaniu: zawis, przelot nad łóżkiem, zasłonięcie ToF; potem teren; monitor ToF wciąż tylko obserwuje (ewentualne wymuszenie lądowania po zebraniu danych) |
| Wysokość bezwzględna nad przeszkodami z samego ToF (`tof_step`) | zaimplementowane na `feat/baro-abs`, nielatane; dalej: reakcja monitora ToF (kontrolowane lądowanie z barometru) i barometr powyżej zasięgu ToF |
| Zabezpieczenie: lądowanie na barometrze przy niewiarygodnym ToF (`kToFUnreliable`) | zaimplementowane na `feat/baro-abs`, nielatane; test: zasłonięcie ToF kartonem w zawisie |
| Podgląd i logowanie telemetrii na telefonie (zamiast laptopa w terenie) | **następne po zakończeniu bieżących testów** (ABS lot 2, test zabezpieczenia `TOF`, test krawędzi na pudle, merge PR #13). Pomysł: pilot tworzy własną sieć Wi-Fi (ESP-NOW do drona zostaje, ten sam kanał), serwuje stronę z podglądem na żywo (wysokość, napięcie, tryb, `hm_trust`, powód failsafe) i odsyła telemetrię przez WebSocket; telefon zapisuje ją do pliku CSV w formacie jak `telemetry_logger.py`, żeby `plot_log.py` działał bez zmian. Sterowanie zostaje na fizycznym pilocie (KILL fizyczny, bez sterowania z ekranu dotykowego). Szacunek: ok. pół dnia. Do ustalenia: bufor lub limit rozmiaru na pilocie, zasięg Wi-Fi, odporność na blokadę ekranu telefonu |
| Łagodny regulator wysokości na barometrze powyżej zasięgu ToF (`feat/soft-altitude`) | **następne po merge PR #13**. Dane z logów: barometr z poprawką podmuchu i filtrem 1–3 s, z offsetem nauczonym w pierwszych 6 s lotu, trzyma się ±0,7 m względem ToF (95% błędów ≤ 0,44–0,65 m). Pomysł: nad progiem 1,0–1,5 m ciąg = trym zawisu z napięcia + powolna korekta z barometru z martwą strefą ±0,5–0,7 m i ograniczoną do ok. ±6% ciągu; bez szybkiej pętli prędkości i bez przełączania filtru. Najpierw symulacja offline (wzmocnienie ciągu 12–16 m/s² na jednostkę gazu, szum i dryf barometru z logów), potem lot w korytarzu na 1–2 m z ToF tylko jako prawdą do oceny, na końcu zewnątrz na uwięzi |
| Martwa strefa obrotu w pilocie przy wysokim gazie | zrobione: pchnięcie lewego drążka do oporu w górę odchylało go w bok o ok. 0,2 (obrót 60–80°/s); teraz przy gazie > 0,5 obrót poniżej 0,3 jest zerowany |
