# 09. Rozwiązywanie problemów

> Dokument rozwijany wraz z projektem. Sekcje o parowaniu, dryfie w locie
> i krótkich lotach zostaną uzupełnione w gałęzi `feat/position-controller`.

## Optical flow

| Objaw | Prawdopodobna przyczyna | Co zrobić |
|---|---|---|
| Przy starcie `PMW3901 not found` | czujnik nie odpowiada na SPI | sprawdź montaż płytki czujnika; ⚠️ pin CS (G12) do weryfikacji ze schematem |
| `flow_squal` stale niskie (< 30), `flow_reject` = 4 | jednolita, błyszcząca lub ciemna podłoga | lepsze oświetlenie, mata z wyraźnym wzorem (np. puzzle piankowe, dywan we wzory) |
| `flow_reject` = 2 lub 3 | wysokość poza oknem 0,10–1,50 m | lataj w oknie lub zmień `kFlowMinHeight/MaxHeight` |
| `flow_reject` = 1 przy poprawnym czujniku | brak pomiaru ToF (zbyt wysoko, ciemna/pochłaniająca powierzchnia) | sprawdź `tof_raw_mm` w telemetrii |
| `vel_x/y` rośnie przy pochylaniu drona w miejscu | błędny znak/zamiana osi flow | powtórz kalibrację (08-testy.md) |
| `pos_x/y` po przeniesieniu o 1 m ≠ 1 m | błędna skala `kFlowRadPerCount` lub wysokość | powtórz kalibrację na stałej wysokości |
| `vel_x/y` dryfuje powoli przy nieruchomym dronie | duży bias akcelerometru; dron nieskalibrowany na poziomej powierzchni | uruchamiaj drona na płaskim, poziomym podłożu (kalibracja „fioletowa”) |
