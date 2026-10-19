# 11. Stable ABS (wysokość z barometru)

Stan: **wyłączone** (`POSHOLD_ENABLE_ABS` = 0 w pilocie i dronie; prawy przycisk przełącza tylko Auto ↔ Manual). Dwa loty
hybrydy zakończyły się upadkiem i uderzeniem w sufit (patrz niżej). Następny krok: łagodny regulator wysokości na
barometrze (`feat/soft-altitude`, patrz [10-plany.md](10-plany.md)). Flagę można włączyć kompilując z `-DPOSHOLD_ENABLE_ABS=1`. Gałąź `feat/baro-abs`. Powód powstania: w parku dolny ToF
został oślepiony, a dron wzniósł się na 20–30 m ([10-plany.md](10-plany.md)).

## Co robi

Trzeci stan **wysokości** w pilocie: prawy przycisk przełącza **Auto → Manual → ABS** (lewy wybiera tryb
sterowania Stable / Sport / Position Hold). ABS to Auto Altitude, w którym wysokość nie pochodzi z dolnego ToF,
tylko z barometru BMP280. Nie wymaga podłogi z teksturą ani ToF-a w zasięgu, więc nadaje się tam, gdzie
Position Hold nie działa (teren otwarty, słońce, przeszkody pod dronem). Pozycji w poziomie nie trzyma.

- ABS łączy się ze Stable i Sport. Position Hold potrzebuje wysokości z ToF, więc wybór Position Hold ustawia
  Auto, a przełączenie wysokości na ABS (lub Manual) w Position Hold przechodzi na Stable.
- Na ekranie ustawień na prawej połowie pojawia się zielony napis `ALT ABS`, a w locie na lewym panelu
  `STB ABS` / `SPT ABS`. Jeśli dron nie ma barometru: `ABS N/A` i `ABS NO BARO`, a dron leci jak Auto z ToF.
- Protokół: flaga `kRcFlagAbsAlt` (bit 1 bajtu 26 pakietu sterującego). Starsze firmware'y ją ignorują
  (lecą z ToF, bez ABS).

## Jak liczona jest wysokość (`abs_height.hpp`)

```
abs = filtr(baro_h, 0,6 s) − poprawka_podmuchu(gaz) + offset
poprawka_podmuchu = 13 Pa · gaz² / 11,5 Pa/m          (ok. 0,6 m przy gazie 0,73)
offset            = uczony z ToF powoli (20 s), tylko gdy ToF zgadza się z abs w granicach 0,30 m
```

- **Barometr** daje wysokość względem ciśnienia zmierzonego na ziemi w trybie parkowania.
- **Podmuch ze śmigieł** obniża ciśnienie na płytce o 6–9 Pa przy zawisie (logi `20261019_194111`
  i `20261019_200020`); stały w zakresie 0,5–1,3 m, więc modelowany jako stała zależna od gazu.
- **Offset** pochłania dryf temperaturowy barometru (ok. 1,2 Pa/K, czyli ok. 10 cm/K) i pogodę.
  Uczy się z ToF tylko gdy oba źródła się zgadzają (|różnica| < 0,30 m). Przeszkoda pod dronem (łóżko,
  stół) lub oślepiony ToF dają większą różnicę, więc **nie zmieniają wysokości**. Dzięki temu zadana
  wysokość jest bezwzględna.
- Na ziemi (tryb parkowania) wysokością jest ToF, więc start jest spójny z dotychczasowym.
- Wynik jest podstawiany do upstreamowego estymatora wysokości (filtr Kalmana z akcelerometrem) w miejscu
  odczytu ToF (`Range`, `sensor.cpp`, hak `[POSHOLD]`). Reszta pętli wysokości (`alt_pid`, `z_dot_pid`)
  pozostaje bez zmian.
- Przy automatycznym lądowaniu blisko ziemi (ToF poprawny i poniżej 0,6 m) znowu używany jest ToF, żeby
  lądowanie zakończyło się poprawnie.

## Dokładność (co wiadomo)

- Pomiar w ręku w stałej temperaturze: ±8 cm przy 0,5 m i ±1 cm przy 2,5 m ([08-testy.md](08-testy.md)).
- W locie szum barometru po filtrze to ok. 0,1–0,2 m; po nauczeniu offsetu błąd względem ToF ±0,1 m.
- Nie wiadomo jeszcze: zachowanie w długim locie przy zmianie temperatury płytki, w nawiewie, wiatr.

## Monitor wiarygodności ToF (`height_monitor.hpp`)

Osobno od Stable ABS działa **obserwator**: porównuje ToF z barometrem (bramka 1,5 m przez 1,5 s) i
loguje `hm_trust` i `hm_pred_m`. Nie steruje jeszcze niczym. Zamierzone zastosowanie: kontrolowane
lądowanie z użyciem wysokości z barometru, gdy ToF jest niewiarygodny w trybach, które z niego korzystają.

## Telemetria

Pola: `baro_pa`, `baro_temp_c`, `tof_status`, `tof_sig_mcps`, `hm_trust`, `hm_pred_m`, `abs_m`.
Ramka telemetrii jest na limicie 58 pól; w zamian zniknęły `ctrl_mode`, `alt_flag`, `alt_mode`,
`flow_rej_total`, `acc_bias_x`, `dist_m`, `r_ref_dps`.

## Plan testów (w mieszkaniu, od najbezpieczniejszego)

1. Stable ABS w zawisie na 0,5 m: porównać `abs_m` z `alt_tof_m` w telemetrii.
2. Przelot nad łóżkiem (53 cm): wysokość nad podłogą ma zostać stała, `alt_tof_m` ma skoczyć w dół.
3. Zasłonięcie ToF kartonem na 1–2 s: dron ma dalej trzymać wysokość.
4. Dopiero po tym: lot w terenie, na uwięzi i z KILL pod ręką.

## Wynik pierwszego lotu (2026-10-19, `20261019_203135.csv`) — nieudany

Dwa starty w Stable + ABS na baterii 4,2 V. Dron mocno falował w pionie (wysokość z ToF 0,04 → 1,1 m w kilka sekund,
gaz 0,53 ↔ 0,75 co ok. 1 s) i wymagał poprawek w poziomie (Stable bez Position Hold).

- Barometr w tym miejscu płytki zmienia ciśnienie o ±6–8 Pa (±0,5–0,7 m) w ciągu pół sekundy, w rytmie gazu.
  Sam model podmuchu (gaz²) nie wystarcza: czujnik ma własne opóźnienie (IIR ×4, oversampling), więc przy zmianie
  gazu ciśnienie reaguje po ok. 0,3 s i regulator dostaje błędną wysokość z opóźnieniem. Pętla wysokości jest
  dostrojona do ToF (szum 5 mm), nie do czujnika z szumem 0,3–0,5 m.
- Na rampie startowej na ziemi (ToF 0,01 m) `abs_m` pokazywało 0,5 m, bo podmuch przy gazie 0,6–0,7 obniża ciśnienie
  mocniej, niż przewiduje model; regulator uznał, że dron jest już w powietrzu.

Wniosek: **wysokość z samego barometru jako wejście do dotychczasowej pętli nie działa**. Możliwe drogi:
(a) osobny estymator z akcelerometrem (filtr komplementarny o niskim przecięciu, barometr tylko koryguje powoli),
(b) barometr tylko jako zabezpieczenie: ToF zostaje głównym źródłem, a monitor wiarygodności przełącza na barometr
tylko do kontrolowanego lądowania. Do czasu wyboru tryb ABS na pilocie jest eksperymentalny.

## Wysokość bezwzględna z samego ToF: kompensacja krawędzi przeszkód (`tof_step.hpp`)

Zamiast (lub obok) barometru: ToF próbkuje co 33 ms, a dron nie zmieni wysokości o więcej niż kilka
centymetrów w jednej próbce, więc skok o więcej niż 15 cm między dwiema kolejnymi próbkami to krawędź
przeszkody (łóżko, stół), a nie wznoszenie. Offset pochłania skok, więc wysokość (nad punktem startu)
pozostaje ciągła i regulator nie zmienia ciągu. Zjazd z przeszkody cofa offset. Pojedynczy błędny odczyt
(skok tam i z powrotem) znosi się sam. Offset jest zerowany na ziemi i przy automatycznym lądowaniu
(lądowanie ma dojść do tego, co dron faktycznie ma pod sobą) i ograniczony do ±1,5 m.

- Działa w Auto Altitude w każdym trybie, także w Position Hold. Optical flow dostaje prawdziwą
  odległość od podłogi (wysokość bezwzględna minus offset), bo skala flow zależy od AGL.
- Monitor wiarygodności porównuje skompensowaną wysokość ToF z barometrem (drugie źródło prawdy).
- Telemetria: `tof_off_m` (offset). Hak `[POSHOLD]` w `sensor.cpp`: wykrycie skoku po filtrze odstających
  (`poshold_tof_step_sample`) i dodanie offsetu do zakresu podawanego estymatorowi.
- Ograniczenie: łagodne zbocze (rampa, schody) nie jest skokiem, więc nie jest kompensowane.
- Test: zawis na ≥ 1,0 m, przelot nad przeszkodą 30–50 cm; `alt_est_m` ma pozostać ciągłe, a `tof_raw_mm`
  skoczyć w dół; `tof_off_m` rośnie o wysokość przeszkody i wraca do zera po przelocie.

### Pierwszy lot nad przeszkodą (2026-10-19, `20261019_205655.csv`, krzesło 45 cm, zawis ok. 0,8–0,9 m)

- Wejście nad siedzisko: ToF 738 → 0 (jedna próbka bez echa) → 360 mm; offset +0,38 m, `alt_est` pozostało ciągłe
  (0,74 → 0,68 m), ciąg prawie bez zmiany. Kompensacja działa.
- Zjazd z krzesła nie został wykryty: ToF rósł łagodnie (340 → 870 mm w 1,8 s), a potem spadł 789 → 669 → 625 → 594 mm
  (pierwszy skok 0,12 m < dawnego progu 0,15 m, kolejne 0,03–0,04 m). Offset został +0,38 m, regulator widział
  zejście z prędkością 1,5 m/s, podniósł ciąg do 0,80 i dron się wzniósł (`alt_est` 1,28 m wobec zadanej 0,82).
- Position Hold wypadł z powodem 4 (flow) nad krzesłem (`squal` 13–30, siedzisko bez tekstury) i przeszedł w Stable.
- Poprawka: próg pojedynczego skoku 0,07 m i reguła pary (dwa kolejne skoki > 0,03 m w tę samą stronę o sumie > 0,12 m).
  Łagodnych zboczy i krzesła z niejednolitą krawędzią nie da się w pełni wykryć progiem; lepszy test na równym pudle.

## Zabezpieczenie: lądowanie na barometrze przy niewiarygodnym ToF

Gdy monitor (`height_monitor`) uznaje ToF za niewiarygodny (rozjazd z barometrem > 1,5 m przez 1,5 s albo brak echa),
nadzorca zatrzaskuje żądanie lądowania z powodem `kToFUnreliable` (12, na pilocie `TOF`) w każdym trybie, w którym
wysokość pochodzi z ToF (nie dotyczy trybu ABS, który z definicji używa barometru). Podczas takiego lądowania
estymator wysokości dostaje wysokość z barometru (`abs_height`, z korektą podmuchu) zamiast ToF; poniżej 0,25 m
(barometr nie jest już wiarygodny) estymator dostaje informację o ziemi, żeby upstreamowe lądowanie się zakończyło.

- Zapobiega scenariuszowi z parku (oślepiony ToF, dron wznosi się bez końca).
- Ryzyko: fałszywy alarm = niespodziewane lądowanie. W dotychczasowych lotach (łącznie ok. 5 minut w zawisie, w tym
  przelot nad krzesłem) `hm_trust` nie spadło.
- Test w mieszkaniu: w zawisie na ok. 0,8 m zasłonić ToF kartonem na 3 s; po ok. 1,5 s `hm_trust` = 0, `ph_reason` = 12,
  lądowanie na barometrze. KILL pod kciukiem.

## ABS v2: akcelerometr trzyma krótki czas, barometr koryguje wolno (do lotów na zewnątrz)

Pierwszy lot wykazał, że barometr z szumem ±0,5 m nie może wchodzić do pętli wysokości z wagą jak ToF (szum 5 mm).
Upstreamowy estymator wysokości to filtr Kalmana z akcelerometrem, z szumem pomiaru wpisanym na stałe dla ToF
(0,004 m). W trybie ABS szum pomiaru ustawiany jest na 0,40 m (`set_measurement_std`, `kAbsKalmanStd`), więc
akcelerometr niesie krótkie zmiany wysokości, a barometr tylko wolno (rzędu kilku sekund) koryguje dryf.

- W trybie ABS limity wysokości są podniesione: zadana do 20 m (`kAbsAltMax`; upstream 1,8 m z ToF), a upstreamowy
  mechanizm wymuszający zejście powyżej 2 m (`Range0flag`) działa dopiero powyżej 20,5 m i nie reaguje na brak
  echa ToF, który na zewnątrz jest normalny.
- Lądowanie w ABS kończy się, gdy wysokość z barometru spadnie poniżej 0,25 m (albo ToF ma poprawny odczyt poniżej 0,6 m).
- Zabezpieczenie `kToFUnreliable` nie działa w ABS (tam ToF nie jest źródłem wysokości).
- Przewidywana dokładność: po nauczeniu się offsetu błąd stały do ok. 0,3 m (podmuch), bez falowania. Do sprawdzenia w locie.
- Pierwsze testy w mieszkaniu (zawis 0,5–1,5 m, porównanie z ToF), potem na zewnątrz **na uwięzi**.

## Drugi lot ABS (v2, `20261019_215025.csv`) — niebezpieczny, wycofane na rzecz hybrydy

Zawis bez dotykania gazu zamienił się w skoki 0 ↔ 2,7 m (KILL po ok. 13 s). Przyczyna: w rampie startowej na ziemi
barometr pokazał −0,34 m (podmuch przy ziemi), a estymator z szumem pomiaru 0,4 m nie nadążał za prawdziwym
wznoszeniem, więc dla regulatora dron był na ziemi (0,03 m) przy pełnym ciągu i wystrzelił do 1,8–2,7 m.
Wniosek: barometr nie może być źródłem wysokości od ziemi.

## ABS v3: hybryda ToF nisko / barometr wysoko

- Źródłem wysokości jest **ToF** (z kompensacją krawędzi), dopóki jest poprawny, monitor mu ufa i wysokość jest
  poniżej progu `kAbsToFMax` (z histerezą +0,2 m). Start, zawis nisko i lądowanie są więc takie jak w Auto.
- Powyżej progu albo gdy ToF jest niewiarygodny (oślepiony, bez echa) źródłem jest **barometr** z akcelerometrem
  (szum pomiaru 0,4 m w estymatorze). Offset barometru jest uczony z ToF-a szybko (3 s), dopóki ToF jest źródłem,
  więc przełączenie jest ciągłe.
- Próg: 2,0 m na zewnątrz; do testu w korytarzu (sufit 2,4 m) tymczasowo **1,0 m**, żeby zobaczyć barometr w akcji,
  gdzie ToF nadal jest prawdą. W trybie ABS limity wysokości są podniesione (20 m), więc w teście nie wychodzić
  drążkiem powyżej ok. 1,5 m.

## Lot 2 hybrydy (2026-10-19, `20261019_223034.csv`, przełączenie ToF → barometr przy 1,0 m) — nieudany

- Poniżej 1,0 m (źródło ToF) zawis jak w Auto: 45 s, wysokość 0,5–0,75 m.
- Powyżej 1,0 m (źródło barometr) estymator odjechał od prawdy o +0,4 m (76–77 s: `est` 2,19 m przy ToF 1,82 m), regulator
  uznał, że dron jest za wysoko, ciąg spadł do 0,55 i dron opadł z 1,8 do 0,2 m (KILL).
- Drugi start: zawis, potem w okolicy progu 1,0 m źródło wysokości przełączało się tam i z powrotem, a przełączanie szumu
  pomiaru (0,004 ↔ 0,4 m) i skok wysokości między źródłami (barometr zawyżony o 0,3–0,5 m) rozkołysały pętlę:
  `est` 0,37 przy ToF 1,25 m, `vz` −2,5 m/s, potem pojedynczy odczyt ToF 5,08 m i `est` 4,5 m (KILL w 101,6 s).

Wniosek: barometr zamontowany w podmuchu śmigieł (±0,4–0,5 m i opóźnienie) nie nadaje się do utrzymywania wysokości
w pętli tego drona, także jako źródło „wysoko”. Zostają: ToF z kompensacją krawędzi, monitor wiarygodności i lądowanie
na barometrze przy niewiarygodnym ToF (to nie wymaga dokładności). Wysokość powyżej zasięgu ToF wymagałaby
barometru poza podmuchem (inna lokalizacja czujnika), GPS albo lotu na stałym ciągu (Manual). Tryb ABS jest
eksperymentalny i nie powinien być używany w locie.
