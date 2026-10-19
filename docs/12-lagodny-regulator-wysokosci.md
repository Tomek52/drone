# 12. Łagodny regulator wysokości na barometrze (projekt)

Stan: **projekt i symulacja, bez kodu w dronie**. Gałąź `feat/soft-altitude`. Cel: lot powyżej zasięgu ToF
(na zewnątrz, 3–20 m) z wysokością trzymaną z dokładnością 1–1,5 m, bez falowania
([11-stable-abs.md](11-stable-abs.md) opisuje trzy nieudane podejścia z barometrem w szybkiej pętli).

## Założenia z pomiarów

| Wielkość | Wartość | Źródło |
|---|---|---|
| Wzmocnienie pionu | 12–16 m/s² na jednostkę gazu | logi `20261017_222945`, `20261019_222946` |
| Szum próbki barometru | ok. 0,12 m (1,4 Pa), opóźnienie 0,3 s | logi stół/podłoga, `20261019_191101` |
| Przesunięcie od podmuchu | 6–9 Pa przy hoverze, ok. 1,4 m na jednostkę zmiany gazu | `20261019_194111`, `200020` |
| Dryf temperaturowy | do ok. 1,2 Pa/K (ok. 1 cm/s podczas nagrzewania) | `20261019_19*`, `20261019_20*` |
| Błąd trymu z napięcia | ≤ 1,4% gazu średnio | logi lotów 2026-10-17 |
| Błąd barometru po filtrze 1–3 s i nauczonym offsecie | ±0,7 m (95% ≤ 0,44–0,65 m) | analiza `200020`, `223034` |

## Regulator

Wysokość z barometru (poprawka na podmuch, offset nauczony w pierwszych sekundach lotu na ToF i potem zamrożony),
filtr dolnoprzepustowy 0,3 s. Powyżej progu przełączenia (1,0–1,5 m):

```
e   = h_zad − h_baro
u   = Kp·martwa_strefa(e, ±0,5 m) + Ki·∫martwa_strefa(e) dt − Kd·v_baro        (|u| ≤ 6–8% gazu)
gaz = trym_zawisu(napięcie) + u
```

`v_baro` to pochodna filtrowana (stała 1 s). Bez szybkiej pętli prędkości, bez przełączania szumu pomiaru
w estymatorze, bez akcelerometru. Wszystkie gazy względem trymu zawisu, który dron już wylicza z napięcia.

## Symulacja (`tools/soft_altitude_sim.py`)

Model pionowy: podwójny integrator z oporem i opóźnieniem silnika 0,1 s, barometr z szumem, opóźnieniem, wpływem
podmuchu i dryfem. Wzmocnienia z przeszukania siatki: **Kp = 0,008 gazu/m, Ki = 0,0003 gazu/(m·s), Kd = 0,08 gazu/(m/s),
filtr 0,3 s, filtr pochodnej 1 s, martwa strefa 0,5 m, autorytet 0,08**.

Błąd względem prawdziwej wysokości w ostatnich 50 s symulacji 160 s (niżej i w tekście: m):

| Scenariusz | RMS | maks. | v RMS [m/s] |
|---|---|---|---|
| zawis 5 m, start z 3 m | 0,36 | 0,45 | 0,01 |
| trym +3% / −3% | 0,38 / 0,17 | 0,52 / 0,33 | 0,02 |
| offset barometru +0,5 m | 0,10 | 0,17 | 0,01 |
| dryf barometru 1 cm/s (cały lot) | 0,97 | 1,29 | 0,02 |
| podmuchy ±0,3 m/s² | 0,77 | 1,39 | 0,26 |
| skok zadanej 5 → 8 m | 0,41 | 0,57 | 0,01 |
| wzmocnienie ciągu 10 / 18 m/s² (opóźnienie 0,2 s) | 0,41 / 0,39 | 0,50 / 0,48 | 0,01–0,02 |

Wnioski: układ jest stabilny we wszystkich scenariuszach, a błąd w granicach 1–1,5 m wynika głównie z dryfu
barometru (dryf 1 cm/s przez cały lot to najgorszy przypadek) i z podmuchów. To mieści się w założonej dokładności.

Ograniczenia modelu: szum barometru jest biały (w rzeczywistości jest skorelowany z turbulencją), wpływ podmuchu
jest liniowy, brak wiatru poziomego, który zmienia ciśnienie dynamiczne. Dopiero loty zweryfikują model.

## Następne kroki

1. Implementacja w `drone/lib/poshold` (czysty C++ z testami jednostkowymi) i hak `[POSHOLD]` w pętli gazu w trybie
   Auto powyżej progu; flaga `POSHOLD_ENABLE_SOFT_ALT` domyślnie wyłączona.
2. Lot w korytarzu na 1–2 m z ToF tylko jako prawdą do oceny (próg przełączenia obniżony do 1,0 m).
3. Dopiero potem zewnątrz na uwięzi i z limitem 5 m.
