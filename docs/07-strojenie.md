# 07. Strojenie regulatorów

> Ta wersja opisuje narzędzia i ogólną procedurę. Konkretne parametry trybu
> Position Hold zostaną opisane w gałęzi `feat/position-controller`.

## Narzędzia

1. Zapis logu: `py tools/telemetry_logger.py --port COMx` (port pilota w locie,
   port drona na stole/uwięzi) — zob. [tools/README.md](../tools/README.md).
2. Wykresy: `py tools/plot_log.py logs/<plik>.csv --groups <grupy>`.
3. Jeden log = jedna zmiana parametru. Zapisuj w notatce: nazwa pliku logu,
   zmieniony parametr, wartość, obserwacje.

## Ogólna kolejność (od wewnątrz kaskady na zewnątrz)

Regulator zewnętrzny stroimy dopiero, gdy wewnętrzny działa dobrze:

1. prędkość kątowa (`rates`) — fabrycznie dostrojona przez M5Stack/Kouhei Ito,
   zwykle nie ruszamy;
2. kąt (`attitude`) — fabrycznie dostrojony;
3. wysokość (`altitude`) — fabrycznie dostrojona dla trybu Auto Altitude;
4. prędkość pozioma i pozycja (`velocity`, `position`) — nasze, stroimy od zera.

## Na co patrzeć na wykresach

| Objaw | Interpretacja | Typowa reakcja |
|---|---|---|
| pomiar oscyluje wokół wartości zadanej z ustaloną częstotliwością | za duże wzmocnienie P (lub D przy szybkich oscylacjach) | zmniejsz P o 20–30% |
| pomiar wolno dochodzi do wartości zadanej, bez przeregulowania | za małe P | zwiększ P o 20–30% |
| stały uchyb, który powoli maleje | za słabe I | zwiększ I (w upstreamie: **zmniejsz** `ti`, bo to stała czasowa) |
| powolne „pływanie” po zwiększeniu I | za duże I | zmniejsz I |
| szum w wartości zadanej wyższego poziomu | za duże D lub zaszumiony czujnik | zmniejsz D / zwiększ filtrację |

Uwaga: w upstreamie regulator PID ma postać `kp, ti, td` (stała całkowania `ti` —
**im większa, tym słabsze całkowanie**).
