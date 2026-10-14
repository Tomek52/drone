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
| `test_telemetry_frame` | kodowanie ramek telemetrii, suma kontrolna, dzielenie schematu na fragmenty, wektory wzorcowe wspólne z Pythonem |

Testy narzędzi w Pythonie (dekoder telemetrii):

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
   dane powinny płynąć przez ESP-NOW (25 Hz).


Pozostałe procedury (uwięź → niski lot) zostaną opisane w gałęziach
`feat/position-estimator` i `feat/position-controller`.
