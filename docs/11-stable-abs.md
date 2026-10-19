# 11. Stable ABS (wysokość z barometru)

Stan: **zaimplementowane, jeszcze nielatane**. Gałąź `feat/baro-abs`. Powód powstania: w parku dolny ToF
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
