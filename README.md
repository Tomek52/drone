# StampFly Position Hold

Firmware drona **M5Stack StampFly v1.0** i pilota **Atom JoyStick (AtomS3)**,
oparty na oficjalnym, otwartym firmware M5Stack (licencja MIT), rozszerzony
o nowy tryb lotu **Position Hold / Auto**: dron wisi w miejscu (XY z czujnika
optical flow PMW3901, Z z czujnika ToF), a drążki przesuwają punkt zadany.

> ⚠️ Projekt w trakcie rozwoju. Aktualny stan — [docs/CHANGELOG.md](docs/CHANGELOG.md).

## Tryby lotu

| Tryb | Przycisk na pilocie | Kolor LED drona |
|---|---|---|
| Stable | lewy przedni, cyklicznie (ekran startowy) | żółty (Manual Alt) / purpurowy (Auto Alt) |
| Sport | lewy przedni, cyklicznie (ekran startowy) | różowy `DC669B` |
| Position Hold | lewy przedni, cyklicznie (ekran startowy); w locie lewy przedni = powrót do Stable | *w przygotowaniu* |
| **KILL** (wyłączenie silników) | **przytrzymaj wciśnięty lewy drążek 0,5 s** — w każdym trybie | — |

Szczegóły: [docs/05-tryby-lotu.md](docs/05-tryby-lotu.md).

## Szybki start (Windows 11, VS Code + PlatformIO)

```powershell
git clone https://github.com/Tomek52/drone.git
cd drone
code stampfly.code-workspace      # otwiera oba projekty w VS Code

pio run -d drone                  # kompilacja firmware'u drona
pio run -d joystick               # kompilacja firmware'u pilota
pio test -d drone -e native       # testy jednostkowe na komputerze
```

Wgrywanie:
- **dron** — przy odłączonej baterii przytrzymaj środkowy przycisk StampS3 i podłącz USB,
  potem `pio run -d drone -t upload`;
- **pilot** — przytrzymaj boczny przycisk AtomS3 ok. 2 s (zielona dioda), potem
  `pio run -d joystick -t upload`.

Pełna instrukcja, parowanie i powrót do firmware'u fabrycznego:
[docs/02-instalacja-i-wgrywanie.md](docs/02-instalacja-i-wgrywanie.md).

## Struktura

| Katalog | Zawartość |
|---|---|
| `drone/` | firmware StampFly (upstream `m5stack/M5StampFly` + nasze zmiany) |
| `joystick/` | firmware pilota (upstream `m5stack/Atom-JoyStick` + nasze zmiany) |
| `tools/` | skrypty Pythona: telemetria, wykresy, analiza logów |
| `docs/` | dokumentacja (po polsku) — zacznij od [docs/README.md](docs/README.md) |

## Bezpieczeństwo

Testy zawsze w kolejności: stół **bez śmigieł** → uwięź → niski lot nad teksturowaną,
dobrze oświetloną podłogą. Używaj okularów ochronnych. Zasady obchodzenia się
z bateriami LiHV: [docs/01-sprzet.md](docs/01-sprzet.md#baterie--zasady-doc).

## Licencja

Kod upstreamu: MIT (M5Stack, Kouhei Ito) — zob. `drone/LICENSE`, `joystick/LICENSE`.
Nasze zmiany: MIT.
