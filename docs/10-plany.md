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
| Wysokość bezwzględna z barometru (tryb Stable ABS) i odporność na oślepienie ToF | po zdarzeniu w parku (dron wzniósł się na 20–30 m po oślepieniu ToF w Position Hold i Stable+AutoAlt, KILL) w toku: etap 1 — telemetria BMP280 i statusu ToF (`feat/baro-abs`), pomiar w ręku w mieszkaniu (stół, łóżko, sufit ok. 2,4 m); etap 2 — fuzja barometr + akcelerometr + ToF i wykrycie złego ToF (kontrolowane lądowanie, tylko w trybach ze ToF); etap 3 — tryb Stable ABS w pilocie. Ryzyko: upstream trzyma ostatni odczyt `Range` przy `RawRange` ≤ 20 (`sensor.cpp`), więc oślepiony czujnik nie daje alarmu |
