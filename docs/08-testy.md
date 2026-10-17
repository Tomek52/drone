# 08. Testy

## Podsumowanie kampanii testowej (2026-10-14 … 2026-10-17)

Sprzęt: StampFly v1.0 + Atom JoyStick (AtomS3), korytarz z płytkami z fugami (dobra tekstura dla optical flow;
pokój z jednolitymi panelami dawał `squal` 8–50 i przechodzenie w Stable). Logi w `logs/`, wpisy parametrów
w [07-strojenie.md](07-strojenie.md), otwarte sprawy w [10-plany.md](10-plany.md).

| Obszar | Wynik |
|---|---|
| Wgrywanie, parowanie, protokół v2, tryby | ✅ działa; po wgraniu dron wymaga restartu zasilania |
| KILL (lewy drążek ≥ 0,5 s) w powietrzu | ✅ działa |
| Kalibracja optical flow | ✅ `swap=true`, `signX=−1`, `signY=+1`, 0,00175 rad/zliczenie, `kFlowAxesVerified=true` |
| Position Hold, poziom (XY) | ✅ odchylenie std. 3–5 cm w zawisie, powrót po popchnięciu 1–4 s, bez kołysania; `kPosKp`, `kVelKp`, `kVelKi` bez zmian |
| Hamowanie po puszczeniu drążka | ✅ `kBrakeTau` 0,30 → 0,45, przeregulowanie za nowy punkt 3–6 cm |
| Wysokość przy dobrej baterii (≥ 3,8 V) | ✅ `alt_kp` 0,50, `z_dot_kp` 0,06; przeregulowanie po starcie 13 cm, odchylenie std. 3–5 cm |
| Wysokość poniżej ok. 3,6 V | ⚠️ wahania ±0,15 m (okres ~4,5 s), przyczyna nieznana (wykluczone: ToF, sprzężenie napięcia, wzmocnienie ciągu) |
| Utrata łączności | ✅ po poprawce: zawis 3 s, `ph_reason` = 7, lądowanie; wcześniej lądował po 0,1 s (błąd nadzorcy, naprawiony) |
| Niska bateria: automatyczne lądowanie | ✅ przy 3,34 V filtrowanego; ⚠️ przy słabszej baterii dron traci wysokość już przy ok. 3,40 V |
| Niska bateria: ostrzeżenie | ⚠️ buzzer drona niesłyszalny w locie, także po zmianie tonu 600 → 2500 Hz (test 2026-10-17); ostrzeżenie w pilocie (pik + `LOW BATT`) dodane, jeszcze nielatane |
| Ochrona OverG (> 2 g) | ✅ za mocne uderzenie w drona wyłącza silniki (upstream, zamierzone) |
| Awaria czujnika po jednym rozruchu (PMW3901 martwy) | ✅ ponowna inicjalizacja 3 razy, komunikat w logu, wymaga cyklu zasilania |

**Ograniczenia, które zostają:**
- Position Hold testowany tylko w korytarzu z teksturowaną podłogą, bez zakłóceń zewnętrznych poza popchnięciem linijką.
- Zawis dobrze trzymany do ok. 3,6 V pod obciążeniem (ok. 3 min lotu od 4,2 V); poniżej tego lepiej lądować ręcznie.
- Dystans z przenoszenia (1 m ±15%) jest nieweryfikowalny ręcznie; skala potwierdzona żyroskopem.
- Dron zawsze latał z baterią 1S LiHV; progi 3,50 / 3,35 V zostawione z firmware'u, na decyzję właściciela.
- Nie sprawdzono: ostrzeżenia w pilocie w locie, lotu dłuższego niż jedna bateria, innych podłóg i oświetlenia,
  yaw z magnetometru, wysokości bezwzględnej nad przeszkodami.

## Testy jednostkowe na komputerze (`native`)

Kod niezależny od sprzętu leży w `drone/lib/poshold/` i jest kompilowany zarówno
na ESP32, jak i na komputerze. Testy (framework Unity) są w `drone/test/`.

```powershell
pio test -d drone -e native              # wszystkie testy
pio test -d drone -e native -f test_math_utils   # wybrany test
```

Na Windows potrzebny jest kompilator GCC w `PATH` (np. MSYS2 MinGW-w64) albo WSL —
zob. [02-instalacja-i-wgrywanie.md](02-instalacja-i-wgrywanie.md).

CI (GitHub Actions, `.github/workflows/ci.yml`) przy każdym PR:
1. kompiluje firmware drona (`esp32-s3-devkitc-1`),
2. kompiluje firmware pilota (`m5stack-atoms3`),
3. uruchamia `pio test -e native` dla drona,
4. uruchamia testy narzędzi Pythona.

| Test | Co sprawdza |
|---|---|
| `test_math_utils` | funkcje pomocnicze: ograniczanie, strefa martwa, zawijanie kąta, obrót 2D |
| `test_position_estimator` | estymator z symulatorem ruchu: znak i skala flow, kompensacja obrotu, wysokość z przechyłem, akcelerometr w układzie heading, bramki jakości z histerezą, estymacja biasu, przerwy flow, całkowanie pozycji z yaw, mapowanie osi |
| `test_rc_protocol` | pakiet v2 (kodowanie, wersja, suma kontrolna, pakiet v1 jako niezgodny), status drona, cykl trybów, wymuszenie Auto Alt, zmiana trybu w locie, kill switch (próg 0,5 s, podtrzymanie) |
| `test_position_controller` | regulator: kierunki kątów, obrót przez yaw, limit 10°, zbieżność w pętli zamkniętej (przeregulowanie < 30%), usuwanie zakłócenia przez człon I, drążki (prędkość, hamowanie, limit wyprzedzenia), geofence, wysokość, strefa martwa, tryb krokowy |
| `test_supervisor` | nadzorca: wejście w tryb, oczekiwanie na start, brak czujnika/kalibracji, timeout, utrata flow z tolerancją i zatrzaskiem, okno wysokości, przełączenie na Stable, utrata łączności (zawis → lądowanie, Manual Alt), bateria (próg, czas, krótkie spadki), geofence, lądowanie |
| `test_telemetry_frame` | kodowanie ramek telemetrii, suma kontrolna, dzielenie schematu na fragmenty, wektory wzorcowe wspólne z Pythonem |

Testy narzędzi w Pythonie (dekoder telemetrii, kalibracja flow):

```powershell
py -m unittest discover -s tools -v
```

## Testy na sprzęcie

### Telemetria (stół, bez śmigieł)

1. Zdejmij śmigła. Podłącz drona kablem USB (bez trybu pobierania) i baterię.
2. `py tools/telemetry_logger.py --list`, potem `--port COMx` dla drona.
3. Po ≤ 2 s powinny pojawiać się wiersze; `lost` ≈ 0, `bad` = 0.
4. Pochyl drona ręcznie — `roll_deg`/`pitch_deg` na wykresie (`plot_log.py --groups attitude`)
   powinny się zgadzać ze znakami w [03-architektura.md](03-architektura.md)
   (roll dodatni = prawa strona w dół, pitch dodatni = nos w górę).
5. Podłącz pilota do PC, uruchom logger na porcie pilota, włącz drona na baterii —
   dane powinny płynąć przez ESP-NOW (zmierzone: ok. 16 Hz, bez luk w `seq`).


### Optical flow — wykrycie czujnika (stół, bez śmigieł)

1. Podłącz drona przez USB, otwórz monitor portu (`pio device monitor -d drone`) i zresetuj drona.
2. W komunikatach startowych musi się pojawić: `[POSHOLD] PMW3901 ready (id=0x49)`.
   - `PMW3901 not found (id=0x.. inv=0x..)` → czujnik nie odpowiada: sprawdź, czy płytka
     czujnika jest na miejscu; ⚠️ pin CS (G12 wg dokumentacji M5Stack) do weryfikacji.
3. Uruchom logger, trzymaj drona 0,3–0,5 m nad podłogą i obserwuj `flow_squal`
   (`plot_log.py --groups flow`). Zanotuj typowe wartości nad Twoją podłogą
   (dywan, panele, płytki) — próg `kFlowMinSqual` = 30 może wymagać korekty.

### Kalibracja optical flow (przenoszenie w ręku)

Cel: wyznaczyć mapowanie osi czujnika na osie drona i skalę. Wymagane przed
pierwszym włączeniem Position Hold.

1. **Śmigła zdjęte.** Dron na USB, `py tools/telemetry_logger.py --port COMx`.
2. Połóż na podłodze taśmę mierniczą. Podłoga teksturowana, dobrze oświetlona.
3. Trzymaj drona **poziomo**, na stałej wysokości ~0,3–0,5 m, bez obracania.
4. Odczekaj 3 s, przesuń drona **do przodu dokładnie o 1,00 m** (ok. 2–3 s), odczekaj 3 s.
5. Przesuń **w prawo dokładnie o 1,00 m**, odczekaj 3 s. Zatrzymaj logger (Ctrl+C).
6. `py tools/plot_log.py logs/<plik>.csv --groups flow velocity` — odczytaj czasy
   początku i końca obu ruchów (wykres `flow_dx/dy` lub `flow_vx/vy`).
7. `py tools/flow_calibration.py logs/<plik>.csv --forward T0 T1 1.0 --right T0 T1 1.0`
8. Wpisz wynik do `drone/lib/poshold/src/poshold_config.hpp`, zbuduj i wgraj firmware.
9. **Weryfikacja:** powtórz przeniesienie do przodu o 1 m — `vel_x_mps` ma być dodatnie
   podczas ruchu, w prawo — `vel_y_mps` dodatnie. ⚠️ `pos_x_m`/`pos_y_m` są w układzie świata
   obróconym o `yaw` (na naszym dronie `yaw` startuje z −180°), więc `pos_x_m` może maleć;
   porównuj moduł przemieszczenia. Odległość z przenoszenia ręcznego ma rozrzut ±40%, dlatego
   **skalę wyznacz z pochyleń** (pkt 10a).
10. **Test kompensacji obrotu:** nad jednym punktem pochylaj drona nosem w górę/dół
    i na boki (±15°) bez przesuwania. `vel_x_mps` i `vel_y_mps` powinny pozostać
    blisko zera (< 0,15 m/s). Duże wartości w fazie z pochylaniem = zły znak osi
    flow (powtórz kalibrację).
10a. **Skala z pochyleń (zalecana, niezależna od ręki):** dla okien z czystym pochyleniem
    `K = Δkąt[rad] / Δzliczeń` (`flow_sum_y` dla pitch, `flow_sum_x` dla roll). Na naszym
    dronie: 8 okien, 0,0015–0,0018, mediana 0,00175 rad/zliczenie.
11. Dopiero po pozytywnej weryfikacji ustaw `kFlowAxesVerified = true`.

### Protokół v2, tryby i kill switch (stół, bez śmigieł)

1. Wgraj **oba** firmware'y z tej samej gałęzi, sparuj.
2. Ekran startowy: lewy przycisk przedni → Stable → Sport → **POS HOLD** (niebieska plakietka)
   → Stable. Przy POS HOLD obrazek wysokości musi pokazywać Auto. Prawy przycisk przy POS HOLD
   → Manual i powrót do Stable.
3. Logger na porcie drona: `rc_proto_ok` = 1, `rc_flight_mode` zgodny z wyborem.
4. **Kill (bez śmigieł!):** tryb Manual Alt, uzbrój drona, lekko dodaj gazu (silniki się kręcą),
   przytrzymaj wciśnięty lewy drążek. Po ~0,5 s silniki muszą stanąć, dron wydaje długi dźwięk,
   pilot pokazuje `KILL`. Krótkie wciśnięcia (< 0,5 s) nie mogą nic robić.
5. **Niezgodność wersji:** wgraj na pilota firmware fabryczny (M5Burner) — dron w parkowaniu
   miga czerwono-niebiesko i piszczy; uzbrojenie nie może być możliwe. Odwrotnie (fabryczny
   dron + nasz pilot) — pilot po ~3 s pokazuje `DRONE FW?`, dron nie startuje.
6. Tryb POS HOLD w tej gałęzi: dron leci jak Stable + Auto Alt, pilot pokazuje `STB NO FW`
   (regulator pozycji dochodzi w `feat/position-controller`).

### Position Hold — test „w ręku” (bez śmigieł!)

Sprawdza kierunki działania regulatora bez ryzyka lotu.

1. **Zdejmij śmigła.** Kalibracja flow wykonana. Logger na porcie drona (USB).
2. Pilot: Position Hold, ekran lotu. Trzymaj drona poziomo ~0,4 m nad podłogą.
3. Start (ekran AtomS3) — silniki kręcą się bez śmigieł, auto-start trwa ~2,5 s.
   Po nim dron powinien zapiszczeć (rosnący dźwięk), LED stały niebieski, pilot `POS HOLD`.
4. Przesuń drona ręką **do przodu** o ~20 cm i trzymaj: `pitch_ref_deg` powinien być
   **dodatni** (nos w górę = hamowanie/powrót). Do tyłu → ujemny.
5. Przesuń **w prawo**: `roll_ref_deg` **ujemny** (powrót w lewo). W lewo → dodatni.
6. Zasłoń czujnik flow dłonią: po ~0,2 s 3 piknięcia, LED miga na niebiesko,
   pilot `STB FLOW` — fallback.
7. Wyłącz silniki (ekran AtomS3 → lądowanie, albo KILL).

Odwrotne znaki w krokach 4–5 = **nie lataj**; sprawdź kalibrację flow
i znaki osi w telemetrii (03-architektura.md).

### Position Hold — uwięź

0. **Przed śmigłami:** bateria naładowana do ≥ 4,2 V (spoczynek). Bez śmigieł 4 sesje po ~20 s
   zbiły napięcie z 4,34 do 3,90 V, a pod obciążeniem ze śmigłami spadek jest większy; przy
   ~3,9 V test kończy się auto-lądowaniem (`kBatteryLandVolt` = 3,35 V) po kilku sekundach.
   Śmigła zamontuj według przewodnika M5Stack (pozycja i kierunek A/B); zdejmij wszystko
   twardego i wrażliwego z promienia, który linki **faktycznie** dopuszczają.
1. Śmigła założone. Przywiąż drona 3–4 cienkimi linkami (np. żyłka) z różnych stron do ciężkiej
   podstawy tak, by mógł się wznieść maksymalnie ~0,4 m i przesunąć ≤ 0,2 m. W Stable dron
   dryfuje i dwie luźne linki to za mało.
2. Najpierw lot w **Stable + Auto Alt** — sprawdź, że dron zachowuje się jak na fabrycznym firmware.
3. Lot w **Position Hold**: po starcie dron powinien sam utrzymać pozycję; łagodne
   wychylenia prawego drążka przesuwają go w kierunku wychylenia, po puszczeniu staje.
4. Test KILL w powietrzu na uwięzi (dron spadnie z niewielkiej wysokości).
5. Test utraty łączności: wyłącz pilota — dron ma wisieć ~3 s, potem łagodnie wylądować.

### Position Hold — niski lot

Teksturowana, dobrze oświetlona podłoga (dywan we wzory, puzzle piankowe), bez
przeciągów, wolna przestrzeń min. 3 × 3 m, okulary ochronne.

1. Start w Position Hold, wysokość ~0,5 m, 30 s zawisu bez drążków — log.
2. Krótkie ruchy prawym drążkiem w każdą stronę, puszczanie — log.
3. Zmiana wysokości lewym drążkiem w zakresie 0,3–1,0 m.
4. Przełączenie w locie na Stable (lewy przycisk przedni) i z powrotem lądowanie.
5. Analiza i strojenie wg [07-strojenie.md](07-strojenie.md).

### Uwięź — wyniki (2026-10-14)

Pierwszy start, Stable + Auto Alt, bateria 3,86 V (zbyt niska, zob. pkt 0), uwięź luźna:

| Obserwacja | Wartość |
|---|---|
| auto-wzlot | gaz 0,19 → 0,77 w ~3 s, oderwanie po ~3 s od startu |
| wysokość zadana / osiągnięta | 0,5 m / **0,69 m** (przeregulowanie ~0,2 m) |
| dryf w Stable (bez drążków) | do **0,48 m/s**, ~0,5 m w 3 s przy `roll` ≈ +1° |
| spadek napięcia pod obciążeniem | 3,86 → 3,54 V przy `thrust` ≈ 0,75 |
| koniec | uderzenie w przeszkodę w zasięgu linek; zabezpieczenie przeciążeniowe upstreamu zatrzymało silniki |

Wnioski: Stable nie utrzymuje pozycji (to zadanie Position Hold), Auto Alt przeregulowuje
o ~0,2 m, więc przy uwięzi 0,5 m ustaw wysokość lewym drążkiem zaraz po starcie na ~0,3 m.

### Uwięź — KILL w powietrzu i utrata łączności (2026-10-17)

Wykonane przez użytkownika na uwięzi, **bez zapisu telemetrii** (brak logu): KILL w powietrzu
zatrzymuje silniki, wyłączenie pilota powoduje zawis i lądowanie. Wynik jest obserwacją, nie
pomiarem: czas zawisu (~3 s), łagodność lądowania i zachowanie `ph_state`/`ph_reason` nie zostały
zmierzone. Do powtórzenia z loggerem przy najbliższym locie.

### Uwięź — KILL w powietrzu (2026-10-17, z logiem `20261017_172112.csv`)

| Obserwacja | Wartość |
|---|---|
| stan przy KILL | `mode` 2 (lot), `ph_state` 1 (oczekiwanie na wejście), wysokość 0,31 m, `vz` +0,38 m/s, gaz 0,60 |
| reakcja | `mode` 2 → 3 (parkowanie) w jednej próbce (~50 ms), `vbat` 4,02 → 4,24 V w tej samej próbce (zdjęte obciążenie silników) |
| po KILL | dron spada na matę (kąty do −66° przy upadku na uwięzi), po spadku spoczywa |

Pierwszy start w tej sesji (7,55 s) skończył się fallbackiem przez geofence po ~3 s: estymata miała
`vx` = 27,6 m/s przeniesione z parkowania (zob. docs/09, „Estymator na ziemi"), a gwałtowne
szarpnięcie uwięzi uruchomiło zabezpieczenie przeciążeniowe. Position Hold nie zdążył wejść
w Active ani w tym, ani w drugim starcie, więc KILL sprawdzony był w trybie oczekiwania na wejście.

### Uwięź — Position Hold po naprawie prędkości na ziemi (2026-10-17, `20261017_174107.csv`)

Bateria 4,08 V przed startem, dron stał nieruchomo 2–3 min w parkowaniu (`vel_x`, `vel_y`, `pos_x`, `pos_y` = 0,000).

| Miara | Wynik |
|---|---|
| wejście w Active | 4,3 s po starcie (7,1 s → 11,4 s) |
| czas w Active | **31 s** (11,4–42,3 s) |
| błąd pozycji względem punktu zadanego | x: std. 2,2 cm (zakres −8…+5 cm); y: std. 7 cm (−34…+26 cm, przy zmianie punktu zadanego drążkiem) |
| wysokość | średnio 0,54 m, std. 4,8 cm, zakres 0,44–0,64 m (przy `z_dot_kp` = 0,06) |
| stały przechył w locie | `roll_ref` średnio −2,3°, rośnie do −3,9° w 20 s (człon całkujący kompensuje stałe zakłócenie w prawo: ciągnięcie uwięzi lub trym) |
| znoszenie w prawo | przed wejściem w Active 0,33 m w 4,3 s (zachowanie Stable); po wejściu pozycja trzymana |
| koniec | `ph_state` 3, `ph_reason` 4 (utrata flow): SQUAL < 30 przez ~0,7 s (19–28), `flow_valid` → 0 po 3 złych próbkach, fallback po 0,2 s zwłoki; potem KILL |

SQUAL w fazie Active: mediana 38, p5 = 28, 7,8% próbek < 30 (`kFlowMinSqual`). `flow_shutter` = 8191
(maksimum) cały czas: za mało światła. Zalecenie docs/07 (próg ≈ 50–60% typowej wartości, tu 19–23)
i lepsze oświetlenie podłogi, zanim zmienimy próg.

### Niski lot bez uwięzi — pierwszy zawis w korytarzu (2026-10-17, `20261017_203405.csv`)

Miejsce: korytarz z płytkami i fugami (kurtyny na ścianach, mata pod dronem), SQUAL w ankiecie 50–106.
W pokoju z jednolitymi panelami SQUAL 8–50 i dwa loty swobodne wypadły w fallback po 0,2–1,4 s.

| Miara | Wynik |
|---|---|
| wejście w Active / czas w Active | 2,8 s po starcie / **19,2 s** (zakończone lądowaniem pilota, bez fallbacku) |
| błąd pozycji XY | std. 3,5 cm (x), 6,7 cm (y); maks. oddalenie od startu 0,28 m |
| wysokość | średnio 0,56 m, std. 7,5 cm |
| SQUAL w locie | mediana 87, minimum 60, 0% poniżej 30 |
| `roll_ref` średnio | −0,76° (na uwięzi −2,3…−3,9°: stały przechył pochodził głównie z uwięzi) |
| napięcie | 4,20 → 3,77 V w 19 s |

Wniosek: jakość flow zależy od podłogi, nie od regulatora. Jednolita podłoga + słabe światło
(`flow_shutter` = 8191) daje SQUAL 8–50 i fallbacki; płytki z fugami dają 60–115.

### Korytarz — odpowiedź na wychylenie drążka (2026-10-17, `20261017_212223.csv`)

Position Hold aktywny 33 s (SQUAL mediana 101, 0% < 30). Sześć wychyleń prawego drążka:

| Zdarzenie | Wychylenie | Prędkość zadana / osiągnięta | Skok punktu zadanego / pozycja |
|---|---|---|---|
| przód, 38,2 s | 0,42 przez 1,05 s | 0,26 / 0,23 m/s, opóźnienie 0,5–0,8 s | −0,01 → −0,15 m; pozycja wychodzi o 5 cm dalej |
| bok, 31,2 s | 0,44 przez 0,59 s | 0,26 / 0,19 m/s | punkt zadany −0,10 m, dron mija go o 8 cm i wraca wolno |

Zadane kąty maks. ~3° (limit 10°): pętla prędkości daleko od nasycenia. Po puszczeniu drążka dron
hamuje dłużej niż zakłada `kBrakeTau` (0,3 s; realnie ~0,45 s), więc mija punkt zadany i wraca.

### Korytarz — zmiany wysokości (2026-10-17, `20261017_212835.csv`)

Active 25,6 s, błąd XY std. 4,3 / 3,0 cm, SQUAL mediana 99. Zadana wysokość zmieniona lewym
drążkiem: 0,50 → 0,60 (osiągnięte 0,64–0,69, błąd ustalony +6…+9 cm) → 0,41 (osiągnięte 0,47 po ~2,5 s,
potem wahania ±8–10 cm, okres 3–4 s). `thrust0` rośnie 0,64 → 0,67 przy spadku napięcia 3,88 → 3,76 V.

### Korytarz — `kBrakeTau` = 0,45 (2026-10-17, `20261017_214337.csv`)

Active 17,5 s, SQUAL mediana 86 (0% < 30). Przeregulowanie za punkt zadany po puszczeniu drążka:
3,2 / 5,4 / 6,5 / 3,3 cm (średnio 4,6 cm, wcześniej 6,5 cm). Błąd XY std. 5,0 / 3,7 cm, w tym ruchy.

Wcześniejsza próba tego samego dnia (`20261017_213815.csv`) nie mogła wejść w Active: po wgraniu
firmware'u czujnik PMW3901 zwracał stałe `squal = 0`, `shutter = 900`, `dx = dy = 0` (martwy po
inicjalizacji); po restarcie zasilania ożył (`squal` 6–9, `shutter` 8191). Przed każdym lotem
sprawdzaj w telemetrii, że `squal` > 0 w postoju.
