# ADR 0006: Nadzorca Position Hold i polityka failsafe

- Status: przyjęta
- Data: 2026-10-14
- Gałąź: `feat/position-controller`

## Kontekst

Position Hold zależy od optical flow, które może stracić wiarygodność (tekstura,
światło, wysokość). Wymagania: automatyczny powrót do Stable + Auto Alt z sygnalizacją,
zawis i łagodne lądowanie po utracie łączności, lądowanie przy niskiej baterii,
twarde limity kątów i odległości od startu, natychmiastowe wyłączenie silników.

## Decyzja

1. **Jeden nadzorca** (`PosHoldSupervisor`, czysty C++, testy `native`) z maszyną stanów
   Idle → WaitEngage → Active → Fallback. Logika bezpieczeństwa jest w jednym miejscu
   i jest testowana bez sprzętu.
2. **Wejście w Position Hold dopiero po auto-starcie upstreamu** i na wysokości ≥ 0,15 m.
   Start odbywa się jak w Stable + Auto Alt (sprawdzona procedura upstreamu).
3. **Fallback jest zatrzaskiwany do lądowania** — bez automatycznego powrotu do Position Hold
   w powietrzu. Pilot po alarmie steruje w Stable; nagłe „przejęcie” drona przez regulator
   po odzyskaniu flow byłoby zaskakujące.
4. **Krótkie zakłócenia flow (< 0,2 s) są tolerowane** — estymator przez ten czas całkuje
   akcelerometr.
5. **Utrata łączności z Auto Alt: zawis 3 s, potem auto-lądowanie upstreamu.** Drążki są
   zerowane (upstream trzymał ostatnie, nieaktualne wartości). Przy Manual Alt — natychmiastowe
   lądowanie jak w upstreamie (zawis bez regulatora wysokości nie jest możliwy).
6. **Niska bateria i geofence wywołują auto-lądowanie upstreamu** (w każdym trybie), żądanie
   jest zatrzaskiwane do lądowania (napięcie rośnie po zmniejszeniu obciążenia).
7. **Geofence dwustopniowy:** punkt zadany nigdy poza 2,0 m; odległość > 2,5 m oznacza awarię
   estymacji lub sterowania → lądowanie.
8. Sygnały dźwiękowe przez nieblokujący odtwarzacz (`alerts`), LED: stały niebieski =
   Position Hold aktywny, migający niebieski = Position Hold utracony.

## Rozważane alternatywy

| Alternatywa | Dlaczego nie |
|---|---|
| Automatyczny powrót do Position Hold po odzyskaniu flow | Nieprzewidywalne dla pilota; ryzyko „walki” pilota z regulatorem. |
| Natychmiastowe lądowanie przy utracie flow | Nad złą podłogą lądowanie bez kontroli XY jest gorsze niż przekazanie sterowania pilotowi. |
| Zawis bez limitu czasu po utracie łączności | Bateria się wyczerpie, dron bez nadzoru. |
| Próg baterii bez filtracji | Chwilowe spadki napięcia przy manewrach wywoływałyby lądowanie. |

## Konsekwencje

- Po fallbacku pilot musi być gotowy sterować w Stable (drążki znów zadają kąty).
- Progi baterii i czasy są w `poshold_config.hpp` i wymagają weryfikacji z logów.
- Geofence działa tylko gdy estymacja jest aktywna (Position Hold).
