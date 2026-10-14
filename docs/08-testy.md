# 08. Testy

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
