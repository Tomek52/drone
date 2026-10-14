# 02. Instalacja, budowanie i wgrywanie (Windows 11)

## Wymagania

| Narzędzie | Do czego | Uwagi |
|---|---|---|
| [Visual Studio Code](https://code.visualstudio.com/) | edytor | |
| Rozszerzenie **PlatformIO IDE** | budowanie, wgrywanie, monitor portu | instaluje PlatformIO Core automatycznie |
| [Git for Windows](https://git-scm.com/download/win) | klonowanie, gałęzie | zalecane `git config --global core.longpaths true` |
| Python 3.10+ | skrypty w `tools/` | `py -m pip install -r tools/requirements.txt` |
| Kompilator GCC dla Windows (np. MSYS2 `mingw-w64-ucrt-x86_64-gcc`) | **tylko** testy `native` | bez niego `pio test -e native` nie zadziała na Windows; alternatywnie WSL |

Sterowniki USB: StampS3 i AtomS3 mają natywny interfejs USB układu ESP32-S3
(„USB Serial/JTAG”, wg specyfikacji M5Stack). Windows 11 ma wbudowany sterownik
klasy CDC, więc zwykle nie trzeba nic instalować — urządzenie pojawia się jako
port `COMx`. ⚠️ do weryfikacji na Twoim komputerze. Jeśli port się nie pojawia,
M5Stack udostępnia stronę „Device USB Driver” w dokumentacji.

Wersje sprzętu: StampFly **v1.0** (StampS3) i Atom JoyStick z **AtomS3** — zob.
[01-sprzet.md](01-sprzet.md).

## Klonowanie

```powershell
git clone https://github.com/Tomek52/drone.git
cd drone
code stampfly.code-workspace
```

Plik `stampfly.code-workspace` otwiera w VS Code oba projekty (`drone/`, `joystick/`)
jako osobne foldery, dzięki czemu PlatformIO widzi oba `platformio.ini`.

## Budowanie

Z paska PlatformIO w VS Code (✓ Build) albo z terminala:

```powershell
pio run -d drone        # firmware drona (środowisko esp32-s3-devkitc-1)
pio run -d joystick     # firmware pilota (środowisko m5stack-atoms3)
```

Pierwsze budowanie pobiera toolchain ESP32-S3 i biblioteki (kilka minut).
Wersje są przypięte w `platformio.ini` — zob. [ADR 0002](decyzje/0002-przypiecie-wersji-toolchaina.md).

## Testy na komputerze (`native`)

```powershell
pio test -d drone -e native
```

Testy nie wymagają drona. Opis testów: [08-testy.md](08-testy.md).

## Wgrywanie na drona (StampFly)

Procedura z oficjalnego przewodnika M5Stack:

1. **Odłącz baterię** — dron musi być wyłączony.
2. **Przytrzymaj środkowy przycisk na module StampS3** i w tym czasie **podłącz kabel USB**.
   Dron wchodzi w tryb pobierania; w Menedżerze urządzeń pojawia się port COM.
3. Wgraj:
   ```powershell
   pio run -d drone -t upload
   ```
   Jeśli PlatformIO wybierze zły port, dopisz w `drone/platformio.ini` w sekcji
   środowiska `upload_port = COM5` (Twój numer portu).
4. Odłącz USB i podłącz baterię (albo naciśnij reset), aby uruchomić nowy firmware.
   ⚠️ Po wgraniu w trybie pobierania ESP32-S3 zwykle nie startuje sam — potrzebny
   jest reset/ponowne zasilenie; do weryfikacji na sprzęcie.

Podgląd komunikatów (z dronem na USB, bez śmigieł):

```powershell
pio device monitor -d drone -b 115200
```

## Wgrywanie na pilota (Atom JoyStick z AtomS3)

Procedura z oficjalnego przewodnika M5Stack:

1. Podłącz AtomS3 kablem USB.
2. **Przytrzymaj boczny przycisk (reset) AtomS3 ok. 2 s**, aż zaświeci się
   **zielona dioda**. Po puszczeniu przycisku dioda gaśnie — AtomS3 jest w trybie
   pobierania. (Krótkie wciśnięcie tego przycisku = zwykły reset.)
3. Wgraj:
   ```powershell
   pio run -d joystick -t upload
   ```
4. Naciśnij krótko boczny przycisk (reset), aby uruchomić firmware.

## Parowanie pilota z dronem

Procedura z oficjalnego przewodnika M5Stack (zgodna z kodem upstreamu):

1. **Przytrzymaj środkowy przycisk AtomS3 (ekran) i włącz pilota.** Pilot wchodzi
   w ekran potwierdzenia parowania.
2. **Wciśnij ekran ponownie**, aby wejść w tryb parowania (pilot zaczyna przeszukiwać
   kanały i cyklicznie piszczy).
3. **Naciśnij reset na StampFly** (lub podłącz baterię). Przez ok. 2,5 s po starcie
   dron rozgłasza swój adres MAC.
4. Pilot zapisuje kanał i adres drona w swojej pamięci flash (SPIFFS), pokazuje ekran
   sukcesu i restartuje się.

Parowanie wykonuje się raz; po wgraniu nowego firmware'u na pilota zapisane dane
zwykle zostają (partycja SPIFFS nie jest nadpisywana przy `upload`). ⚠️ do weryfikacji.

## Powrót do fabrycznego firmware'u (M5Burner)

1. Pobierz [M5Burner](https://docs.m5stack.com/en/download) dla Windows i uruchom.
2. Po lewej wybierz typ urządzenia **STAMPS3**.
3. Pobierz **StampFly Firmware** (dron) i **StampFly Controller Firmware** (pilot).
4. Wprowadź urządzenie w tryb pobierania (jak wyżej), kliknij **Burn**, wybierz port.
5. Po wgraniu obu firmware'ów wykonaj ponownie parowanie.

Alternatywnie upstream zawiera gotowe pliki `.bin` w `drone/firmware/` i
`joystick/firmware/`. ⚠️ Adres, pod który należy je wgrać (np. `0x0` dla obrazu
scalonego), nie jest opisany w upstreamie — zalecany jest M5Burner.

## Aktualizacja z upstreamu (M5Stack)

Repozytorium zawiera oba projekty upstream jako git subtree
(zob. [ADR 0001](decyzje/0001-struktura-repozytorium.md)). Jednorazowo:

```powershell
git remote add upstream-drone https://github.com/m5stack/M5StampFly
git remote add upstream-joystick https://github.com/m5stack/Atom-JoyStick
```

Dron (cały upstream w `drone/`):

```powershell
git checkout -b chore/upstream-sync main
git subtree pull --prefix=drone upstream-drone main
```

Pilot (w upstreamie projekt leży w `examples/StampFlyController`, więc najpierw
trzeba go wyodrębnić — `subtree split` jest deterministyczny, więc historia się zgadza):

```powershell
git clone https://github.com/m5stack/Atom-JoyStick ..\Atom-JoyStick
git -C ..\Atom-JoyStick subtree split --prefix=examples/StampFlyController -b stampflycontroller
git fetch ..\Atom-JoyStick stampflycontroller
git subtree merge --prefix=joystick FETCH_HEAD
```

Konflikty mogą pojawić się tylko w miejscach oznaczonych komentarzem `[POSHOLD]`
(nasze „haczyki” w kodzie upstreamu).
