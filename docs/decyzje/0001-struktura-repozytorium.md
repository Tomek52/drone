# ADR 0001: Jedno repozytorium z dwoma projektami upstream jako git subtree

- Status: przyjęta
- Data: 2026-10-14
- Gałąź: `main` (import), `chore/repo-setup` (dokumentacja)

## Kontekst

Firmware drona pochodzi z `m5stack/M5StampFly` (całe repozytorium to projekt
PlatformIO), a firmware pilota z `m5stack/Atom-JoyStick`, gdzie projekt PlatformIO
leży w podkatalogu `examples/StampFlyController`. Chcemy:

- jedno repozytorium z katalogami `drone/` i `joystick/` (zmiany protokołu dotyczą
  obu projektów i powinny trafiać w jednym PR),
- zachować historię upstreamu i móc zaciągać poprawki od M5Stack,
- nie wymagać od użytkownika znajomości submodułów.

## Decyzja

- `drone/` — `git subtree add --prefix=drone upstream-drone main` (pełna historia).
- `joystick/` — `git subtree split --prefix=examples/StampFlyController` na klonie
  upstreamu, a następnie `git subtree add --prefix=joystick` (historia tego katalogu).
- Dwa remote'y: `upstream-drone` i `upstream-joystick` (jeden remote `upstream` nie
  może wskazywać na dwa różne repozytoria).
- Nasz kod w osobnych plikach (`drone/lib/poshold/`, nowe pliki w `src/`);
  w plikach upstreamu tylko krótkie wywołania z komentarzem `[POSHOLD]`.
- Procedura aktualizacji: [02-instalacja-i-wgrywanie.md](../02-instalacja-i-wgrywanie.md#aktualizacja-z-upstreamu-m5stack).

## Rozważane alternatywy

| Alternatywa | Dlaczego nie |
|---|---|
| Fork `m5stack/M5StampFly` | Nie mieści projektu pilota; protokół trzeba by zmieniać w dwóch repozytoriach. |
| Submoduły git | Wymagają `--recursive`, łatwo o rozjechane wersje; zmiany w kodzie upstreamu musiałyby żyć w osobnych forkach. |
| Kopia plików bez historii | Utrata historii i trudne porównywanie z upstreamem. |
| Subtree całego `Atom-JoyStick` | Projekt leżałby w `joystick/examples/StampFlyController` — niewygodne ścieżki. |

## Konsekwencje

- Repozytorium zawiera historię upstreamu (w tym pliki `.bin` z firmware'em fabrycznym,
  ok. 8 MB w `.git`).
- Aktualizacja pilota wymaga kroku `subtree split` na klonie upstreamu (opisane w dokumentacji).
- Workflowy CI upstreamu w `drone/.github` i `joystick/.github` (jeśli są) nie są
  uruchamiane przez GitHub — działa tylko `/.github/workflows/ci.yml`.
