# Kamera FPV Eachine TX06 na StampFly v1.0 — analiza integracji

Kamera: **Eachine TX06** (AIO: kamera 700TVL + nadajnik 5,8 GHz 25 mW, 48 kanałów).
Odbiór: gogle **BETAFPV VR03** (analog 5,8 GHz, 48 kanałów, skanowanie przyciskiem).
Dron: **StampFly v1.0 (K138)**, bateria 1S LiHV 300 mAh, masa z baterią ok. 36,8 g.

Ten dokument to analiza i plan. **Kod firmware'u nie został zmieniony** — propozycje zmian
są w sekcji 11 jako diff do akceptacji.

Oznaczenia pewności: **SCH** = odczytane ze schematu M5Stack (strony schematów w oficjalnym
PDF dokumentacji StampFly, zob. „Źródła”), **KOD** = kod w tym repozytorium,
**OFERTA** = opis sprzedawcy (niepotwierdzony), **⚠️** = do zmierzenia na sprzęcie.

## Spis treści

1. [Specyfikacja TX06](#1-specyfikacja-tx06)
2. [Zasilanie StampFly i punkty podłączenia](#2-zasilanie-stampfly-i-punkty-podłączenia)
3. [Zakłócenia i filtr](#3-zakłócenia-i-filtr)
4. [Bilans masy i energii](#4-bilans-masy-i-energii)
5. [INA3221 i firmware](#5-ina3221-i-firmware)
6. [Mocowanie](#6-mocowanie)
7. [Kanały 5,8 GHz w UE](#7-kanały-58-ghz-w-ue)
8. [Lista elementów (BOM)](#8-lista-elementów-bom)
9. [Kroki montażu](#9-kroki-montażu)
10. [Ryzyka](#10-ryzyka)
11. [Propozycje zmian w firmware (diff)](#11-propozycje-zmian-w-firmware-diff)
12. [Czego nie udało się ustalić](#12-czego-nie-udało-się-ustalić)

## Podsumowanie

- **Zasilanie:** z szyny **`VBAT_IN`** (za tranzystorem zabezpieczającym Q1 i za bocznikiem
  INA3221), przez filtr LC 4,7 µH + 47 µF. Nie z 3,3 V (brak danych o zapasie regulatora
  i za niskie napięcie dla TX06) ani z Grove (tam jest `VBAT_IN` minus spadek na diodzie
  Schottky'ego — za mało pod koniec lotu).
- **Masa:** +3,9 g (kamera, przewody, filtr, uchwyt TPU) → ok. +16% mocy w zawisie,
  czas lotu ok. **4 min → ok. 3,2 min**, stosunek ciągu do masy przy 3,7 V ok. **2,1 → 1,9**.
- **Pomiar:** bocznik INA3221 to **R12 = 10 mΩ** na kanale 2 (SCH), a biblioteka ma
  domyślnie 10 mΩ — prąd baterii (razem z kamerą) można odczytać od razu przez
  `ina3221.getCurrent(INA3221_CH2)`. Firmware dziś czyta tylko napięcie.
- **Kanały legalne w UE (5725–5875 MHz, 25 mW e.i.r.p.):** np. **F2–F7, R4–R6, A2–A7, B2–B7**.
  Pasmo E w całości poza zakresem.

## 1. Specyfikacja TX06

| Parametr | Wartość | Pewność | Jak sprawdzić |
|---|---|---|---|
| Zasilanie | 3,3–5,5 V | OFERTA (Banggood) | zasilacz laboratoryjny: obniżaj napięcie od 4,2 V do zaniku obrazu w goglach; zapisz próg (spodziewany ≤ 3,3 V) |
| Pobór prądu | ok. 280 mA „typowo” | OFERTA, bez podanego napięcia | multimetr szeregowo przy 3,3 / 3,7 / 4,2 V, po 2 min pracy (nadajnik się nagrzewa) |
| Masa | 2,8 g bez uchwytu | OFERTA | waga jubilerska 0,01 g, z przewodami po skróceniu |
| Wymiary | 14 × 13,8 mm (moduł) | OFERTA | suwmiarka (do projektu uchwytu) |
| Moc nadajnika | 25 mW, pit mode | OFERTA | — |
| Kanały | 48 (6 pasm) | OFERTA; nazwy pasm w źródłach niespójne | tabela kanałów z instrukcji w pudełku |
| Przewody | czerwony = +, czarny = GND, żółty = wyjście wideo, biały/zielony = SmartAudio | **⚠️ niepotwierdzone** (agregator instrukcji manuals.plus, forum) | zobacz opis na płytce; zmierz omomierzem, który przewód ma ciągłość z masą (ekranem obiektywu/metalową obudową); pierwsze włączenie z zasilacza z limitem prądu 0,4 A |
| Złącza | Molex 1,25 mm 2P (zasilanie), JST-SH 1,0 mm 4P | ⚠️ agregator | oględziny |

**Nie podłączamy** wyjścia wideo ani SmartAudio — StampFly nie ma OSD, a kanał zmienia się
przyciskiem na kamerze. Te przewody skróć i zaizoluj (lub odetnij przy złączu, jeśli
na pewno nie będą potrzebne).

## 2. Zasilanie StampFly i punkty podłączenia

### Co jest na schemacie (SCH)

Płytka przednia (ToF przedni + złącze baterii) i płyta główna:

```
 Płytka przednia                       Płyta główna
 ┌──────────────────────┐             ┌──────────────────────────────────────────────────────────────┐
 │ J1 BAT-2p@2mm        │             │                                                              │
 │  BAT+ ──┬── VBAT ────┼── P1/P3 ────┼─ VBAT ─ Q1 AP50P20Q ──┬── R12 0,01 Ω ──┬── VBAT_IN ───────────┐ │
 │         │            │ (2×4, 2 pin)│  (P-MOSFET, ochrona   │   (bocznik)    │                     │ │
 │         C3 10µF      │             │   przed odwrotną      │ IN+2 (CH2P)    │ IN−2 (CH2N)         │ │
 │         HT7533 → 3,3V│             │   polaryzacją)        │ INA3221 kanał 2 (prąd całego drona)  │ │
 │         (tylko ToF   │             │            C20,C21 22µF               C22,C23 22µF          │ │
 │          przedni+LED)│             │                                                            │ │
 │  BAT− ── GND ────────┼─────────────┼─ GND                   ┌─── silniki J1/J4/J5/J6: pin1 = VBAT_IN,
 └──────────────────────┘             │                        │    pin2 = dren SI2302 (NIE masa!), 100 nF
                                      │   VBAT_IN ─────────────┤
                                      │                        └─── D3 B5819W (Schottky) ── +5VOUT ──┬── StampS3 „M5V” (wejście 5 V modułu)
                                      │                                                             ├── Grove czerwony J3 pin 3 (VCC)
                                      │                                                             ├── Grove czarny  J2 pin 3 (VCC)
                                      │                                                             └── P4 pin 2 „5V” (PMW3901)
                                      │   +3,3 V płyty głównej: z modułu StampS3 (P2 pin 11) — regulator wewnątrz modułu
                                      └──────────────────────────────────────────────────────────────┘
```

Wnioski ze schematu:

- **Nie ma wyłącznika zasilania.** Dron włącza się po włożeniu baterii. Jedyne „zabezpieczenie”
  to Q1 (ochrona przed odwrotną polaryzacją). **Nie ma bezpiecznika** — zwarcie na `VBAT_IN`
  to prąd ograniczony tylko rezystancją baterii (kilkanaście amperów).
- **„+5VOUT” nie ma 5 V.** To `VBAT_IN` minus spadek na diodzie B5819W (ok. 0,3–0,4 V przy
  kilkuset mA). Nazwa jest myląca.
- **Bocznik INA3221 = R12 = 10 mΩ, kanał 2.** Napięcie szyny mierzone jest na wejściu IN−2,
  czyli na `VBAT_IN` (za Q1 i za bocznikiem). Przez bocznik płynie prąd **całego drona**:
  silniki, StampS3, czujniki.
- **Silniki:** przełączanie po stronie masy (SI2302), PWM 150 kHz (KOD: `freq = 150000`
  w `drone/src/flight_control.cpp`), tylko 100 nF na silniku, **brak diod gaszących**
  na schemacie → szpilki napięcia na `VBAT_IN`. Obok J6 jest dopisek „4.2V@2.23A”
  (prawdopodobnie parametr silnika).

### Opcje podłączenia

| Opcja | Napięcie dla kamery | Plusy | Minusy | Ocena |
|---|---|---|---|---|
| **A. `VBAT_IN`** (za Q1 i bocznikiem), przez filtr LC | 3,3–4,35 V minus ~0,04 V na dławiku | najwyższe dostępne napięcie; prąd kamery wchodzi do pomiaru INA3221; ochrona Q1 | najbardziej zaszumiona szyna (silniki) → filtr; brak bezpiecznika → staranna izolacja; trzeba znaleźć punkt lutowniczy | **zalecana** |
| B. `VBAT` przed Q1 (pady złącza baterii na płytce przedniej) | jak A + ~0,05 V | łatwy dostęp, odrobinę wyższe napięcie | **kamera poza pomiarem prądu**; brak ochrony Q1; ten sam szum | dopuszczalna, gorsza |
| C. Grove (VCC = `+5VOUT`) | `VBAT_IN` − 0,3…0,4 V | gotowe złącze, bez lutowania na płycie | przy 3,6 V baterii kamera dostaje ok. 3,2–3,3 V → **zanik obrazu w drugiej połowie lotu**; dioda D3 (1 A) dzieli prąd z wejściem StampS3 — skok poboru kamery obniża zasilanie ESP32 | odrzucona |
| D. +3,3 V | 3,3 V (dolna granica TX06) | stabilne napięcie | regulator jest **w module StampS3** — jego typu i zapasu prądu nie ma na schemacie; ESP32 w szczytach Wi-Fi/ESP-NOW pobiera kilkaset mA; +280 mA kamery może przeciążyć LDO i **zresetować ESP32**; ciepło | odrzucona |
| E. HT7533 na płytce przedniej | 3,3 V | — | HT7533 to mały LDO (rodzina 100 mA wg producenta); zasila ToF przedni | odrzucona |

**Rekomendacja: opcja A.** Uzasadnienie: jedyna opcja, która daje kamerze napięcie powyżej
jej minimum (3,3 V) przez cały użyteczny lot (do progu lądowania 3,35 V), nie obciąża
zasilania ESP32 i zostawia prąd kamery w pomiarze INA3221.

**Punkt lutowania (⚠️ do znalezienia na płytce):** schemat nie pokazuje rozmieszczenia
elementów. `VBAT_IN` jest m.in. na **pinie 1 każdego złącza silnika** i na padach C22/C23.
Jak sprawdzić multimetrem (bateria włożona, śmigła zdjęte):

1. Plus: punkt, który ma napięcie baterii i **nie zmienia się**, gdy silnik się kręci
   (pin 2 złącza silnika ma wtedy przebieg PWM — to nie jest masa ani plus!).
2. Masa: weź z **pinu 4 złącza Grove** (GND) albo z masy złącza baterii. **Nigdy z pinu 2
   złącza silnika.**
3. Kontrola: napięcie między wybranymi punktami ≈ `vbat_v` z telemetrii.

## 3. Zakłócenia i filtr

Źródło: 4 silniki szczotkowe, PWM 150 kHz, bez diod gaszących, ok. 3–4 A w zawisie.
Objawy w obrazie analogowym: poziome pasy/„rolki” zmieniające się z gazem.

TX06 jest projektowana pod drony 1S i zwykle ma własny filtr wejściowy. **Zacznij bez
filtra tylko na stole**; jeśli przy kręcących się silnikach (bez śmigieł) widać pasy, dodaj
filtr. Na drona i tak polecam filtr od razu — waży ok. 0,1 g.

```
VBAT_IN ──── L1 4,7 µH ────┬──────────┬──────────┬───── TX06 (+) czerwony
                           │          │          │
                         C1 47 µF   C2 22 µF   C3 100 nF
                         X5R 6,3 V  tantal     X7R
                         (0805/1206) (case A)  (0402/0603)
                           │          │          │
GND (Grove pin 4) ─────────┴──────────┴──────────┴───── TX06 (−) czarny
```

| Element | Wartość | Uwagi | Masa (szac.) |
|---|---|---|---|
| L1 | 4,7 µH, **Isat ≥ 0,8 A**, **DCR ≤ 0,15 Ω** | dławik mocy SMD 2520 (1008) lub podobny; spadek ≤ 0,28 A × 0,15 Ω ≈ 0,04 V | ~0,03 g |
| C1 | 47 µF, X5R, ≥ 6,3 V | ceramika; przy 4 V efektywnie ~20 µF (DC bias) | ~0,02–0,05 g |
| C2 | 22 µF tantal, 6,3 V, ESR ok. 1–3 Ω | **tłumi rezonans LC** (ceramika ma bardzo niski ESR) | ~0,03 g |
| C3 | 100 nF X7R | jak najbliżej kamery | <0,01 g |

Obliczenia: f₀ = 1 / (2π√(LC)). Dla L = 4,7 µH i C = 47 µF: f₀ ≈ 10,7 kHz; przy efektywnych
20 µF: f₀ ≈ 16 kHz. Tłumienie przy 150 kHz (filtr 2. rzędu, 40 dB/dekadę): (150/16)² ≈ 80×,
czyli ok. **38 dB** (do 46 dB przy pełnych 47 µF). Impedancja charakterystyczna √(L/C) ≈ 0,3–0,5 Ω,
dlatego ESR tantalu ~1 Ω daje dobre tłumienie rezonansu.

Montaż filtra: elementy przylutowane „w powietrzu” na końcu przewodu przy kamerze,
zabezpieczone koszulką termokurczliwą, albo na kawałku płytki uniwersalnej 5 × 8 mm.
Przewody zasilające skręć ze sobą (mniejsza pętla = mniej zakłóceń indukowanych).

Dodatkowo (opcjonalnie, nie wymaga zmian w kamerze): kondensator 100 nF bezpośrednio na
zaciskach każdego silnika już jest (C1/C3/C12/C13 na schemacie); jeśli zakłócenia zostaną,
pomaga koralik ferrytowy na przewodzie kamery lub rozdzielenie prowadzenia przewodów kamery
od przewodów silników.

## 4. Bilans masy i energii

### Założenia

| Wielkość | Wartość | Źródło |
|---|---|---|
| Masa drona z baterią m₀ | 36,8 g | podana przez Ciebie (DOC: 27,7 g bez baterii) |
| Dodatek Δm | 2,8 g kamera + 0,3 g przewody i filtr + 0,8 g uchwyt TPU = **3,9 g** | szacunek |
| Czas lotu bez kamery | ok. 4 min (DOC), Position Hold dobrze trzymany do 3,6 V ≈ 3 min (`08-testy.md`) | DOC, nasze testy |
| Prąd w zawisie I₀ | ≈ 0,85 × 300 mAh / 4 min ≈ **3,8 A** | szacunek (85% pojemności użyteczne) ⚠️ zmierzyć przez INA3221 |
| Prąd kamery I_c | 0,28 A | OFERTA ⚠️ |
| Trym zawisu | `duty₀(V) = −0,2448·V + 1,5892` | KOD: `get_trim_duty()` w `drone/src/flight_control.cpp` |

### Moc i czas lotu

Z teorii pędu (stała sprawność śmigła i silnika) moc w zawisie ∝ m^1,5:

- m₁ = 40,7 g, m₁/m₀ = 1,106 → moc silników × 1,106^1,5 = **× 1,163**
- kamera: 0,28 / 3,8 = **+0,074** (względem prądu zawisu)
- łącznie: **× 1,237** → czas lotu × 0,81

| | bez kamery | z kamerą |
|---|---|---|
| lot do wyczerpania | ~4,0 min | **~3,2 min** |
| Position Hold do 3,6 V | ~3,0 min | **~2,4 min** |

### Zapas ciągu

Przybliżenie: ciąg ∝ (wypełnienie PWM × napięcie)², więc potrzebne wypełnienie rośnie
jak √(m₁/m₀) = **× 1,052**. Stosunek ciągu do masy ≈ 1 / duty² (przy 100% PWM):

| Napięcie | duty zawisu bez kamery | z kamerą | T/W bez | T/W z kamerą |
|---|---|---|---|---|
| 4,20 V | 0,561 | 0,590 | 3,2 | 2,9 |
| 3,70 V | 0,683 | 0,719 | 2,1 | 1,9 |
| 3,50 V | 0,732 | 0,770 | 1,9 | 1,7 |
| 3,35 V | 0,769 | 0,809 | 1,7 | 1,5 |

Model jest zgrubny (nieliniowość silnika, opory, spadek napięcia pod obciążeniem), ale
pokazuje, że **lot jest możliwy z rozsądnym zapasem**, a pod koniec baterii zapas spada do
ok. 1,5. Weryfikacja: `thrust_duty` w zawisie z telemetrii (`tools/plot_log.py`).

**Ważne dla Auto Alt:** trym `get_trim_duty()` nie wie o dodatkowej masie. Z kamerą
potrzebne będzie ok. +5% wypełnienia ponad trym; regulator wysokości działa w zakresie
±15% wokół trymu (`03-architektura.md`), więc kamera „zje” ok. 1/3 jego zakresu. Patrz
propozycja 11.2.

## 5. INA3221 i firmware

### Co jest w kodzie (KOD)

| Plik | Miejsce | Co robi |
|---|---|---|
| `drone/src/sensor.cpp` | `INA3221 ina3221(INA3221_ADDR40_GND);` (l. 36) | obiekt czujnika, adres 0x40 |
| `drone/src/sensor.cpp` | `sensor_init()`: `ina3221.begin(&Wire1); ina3221.reset();` (l. 144–146) | domyślna konfiguracja biblioteki: bocznik **10 mΩ** na każdym kanale (zgodny z R12) |
| `drone/src/sensor.cpp` | `sensor_read()`: `Voltage = ina3221.getVoltage(INA3221_CH2);` (l. 357–366) | **tylko napięcie kanału 2** (`VBAT_IN`), filtr, licznik `Under_voltage_flag` |
| `drone/src/flight_control.hpp` | `POWER_LIMIT 3.34`, `UNDER_VOLTAGE_COUNT 100` (l. 47–48) | próg upstreamu — **tylko kolor LED** (`drone/src/led.cpp`, l. 78–86) |
| `drone/lib/poshold/src/poshold_config.hpp` | `kBatteryWarnVolt 3.50`, `kBatteryLandVolt 3.35`, `kBatteryLandTime 2.0` (l. 135–138) | nasze progi: ostrzeżenie i automatyczne lądowanie |
| `drone/src/poshold_glue.cpp` | `in.vbat = Voltage;` | napięcie do nadzorcy |
| `drone/src/telemetry_stream.cpp` | pole `vbat_v` | telemetria napięcia |
| `drone/src/flight_control.cpp` | `get_trim_duty(float voltage)` | trym zawisu zależny od napięcia |

Kanały INA3221: **używany tylko kanał 2**. Kanały 1 i 3 mają niepodłączone wejścia (SCH).

### Odpowiedzi

- **Czy da się mierzyć prąd kamery?** Osobno — **nie**, bo kanały 1 i 3 nie mają bocznika
  ani połączeń (trzeba by dolutować bocznik i przewody do nóżek układu QFN — nie polecam).
  Przy opcji A prąd kamery jest częścią prądu kanału 2. Prąd kamery wyznaczysz na ziemi:
  różnica `ibat` z kamerą i bez (silniki wyłączone), albo multimetrem przed montażem.
- **Prąd całkowity:** `ina3221.getCurrent(INA3221_CH2)` daje amper od razu (biblioteka
  liczy z bocznikiem 10 mΩ). Zakres: ±163,8 mV / 10 mΩ = **±16 A**, rozdzielczość 40 µV =
  **4 mA**. Rezystory 10 Ω w wejściach (R7, R16) wprowadzają niewielki błąd offsetu;
  biblioteka ma `getCurrentCompensated()` i `setFilterRes()`, ale to wymaga kalibracji.
  Propozycja 11.1 dodaje pole telemetrii.
- **Wpływ na odczyty i progi:** `Voltage` to napięcie `VBAT_IN`, czyli baterii minus spadek
  na rezystancji wewnętrznej baterii, Q1 i R12. Dodatkowe 0,28 A obniży je o
  0,28 A × R_wew (⚠️ R_wew nieznane; dla małego ogniwa rzędu 0,1–0,3 Ω → **0,03–0,08 V**)
  plus efekt większego prądu silników (cięższy dron). Skutek: progi **3,50 V / 3,35 V
  zadziałają wcześniej** — to właściwe zachowanie (chronią baterię), progów nie trzeba
  zmieniać przed lotami z logami. Uwaga: **minimum kamery 3,3 V jest tuż pod progiem
  lądowania 3,35 V** — przy szpilkach spadku napięcia pod koniec lotu obraz może na chwilę
  zgasnąć (patrz Ryzyka).

## 6. Mocowanie

Rozkład elementów (DOC — rysunek wymiarowy i zdjęcia): moduł StampS3 stoi pionowo na
środku płyty, na nosie jest pionowa płytka z przednim ToF i złączem baterii, bateria leży
pod korpusem, **PMW3901 i dolny ToF patrzą w dół** z dolnej strony.

| Wymaganie | Rozwiązanie |
|---|---|
| Pole widzenia | kamera na górze, **z przodu**, nad płytką nosową, obiektyw wysunięty przed osłony śmigieł tylko tyle, żeby nie widzieć ram (120° FOV) |
| Kąt | **5–10° w górę**. Position Hold ogranicza przechył do 10°, loty są wolne, więc większy kąt (typowe 15–25° w whoopach) dawałby widok „w sufit” |
| Przedni ToF (VL53L3) | **nie zasłaniać** jego okna (stożek ok. ±12,5°); uchwyt i przewody nad płytką, nic poniżej górnej krawędzi płytki nosowej przed czujnikiem |
| Dolny ToF i PMW3901 | nic nie może zwisać pod spód (przewody prowadzić górą) |
| Środek ciężkości | 3,9 g ok. 30 mm przed środkiem → przesunięcie SC o ok. **2,9 mm do przodu** (3,9 g × 30 mm / 40,7 g). Regulator kąta skompensuje to członem całkującym, ale lepiej: kamera jak najbliżej środka; jeśli trzeba, bateria przesunięta o kilka mm do tyłu |
| Antena nadajnika | pionowo, **jak najdalej od anteny StampS3** (górny koniec modułu) i od magnetometru BMM150 (w firmware nieużywany — yaw z żyroskopu) |
| Ciepło | nadajnik się grzeje: uchwyt nie może go szczelnie obejmować; zostaw szczelinę na przepływ powietrza |

### Założenia do uchwytu z TPU

- Materiał TPU 95A (amortyzuje uderzenia, nie pęka); wypełnienie 100%, ścianki 0,8–1,2 mm;
  masa docelowa **≤ 0,8 g**.
- Mocowanie **bez kleju do płytki**: obejma zatrzaskiwana na pionowej płytce nosowej albo
  na dwóch słupkach/ramionach ramy przy przednich osłonach (wymiary zdejmij suwmiarką
  ⚠️ — rysunek DOC podaje tylko gabaryty: 81,5 × 81,5 × 31 mm, rozstaw silników 65 mm
  po przekątnej).
- Gniazdo kamery 14 × 13,8 mm z luzem 0,2 mm, wcisk; klin dający **7,5° ± 2,5°**
  (możesz wydrukować wersje 5° i 10°).
- Otwór/kanał na przewody, uchwyt na antenę (pionowo do góry).
- Brak elementów poniżej linii okna przedniego ToF.
- Druk w orientacji, w której klin i gniazdo nie wymagają podpór.

## 7. Kanały 5,8 GHz w UE

Zakres dla urządzeń bliskiego zasięgu bez licencji: **5725–5875 MHz, 25 mW e.i.r.p.**
(CEPT ERC/REC 70-03, załącznik 1). Analogowy sygnał wideo zajmuje ok. 20 MHz, więc
bezpiecznie są kanały z częstotliwością środkową **5735–5865 MHz** (margines 10 MHz od
krawędzi). Tabela standardowych pasm (tabela TX06 ⚠️ do porównania z instrukcją
z pudełka — źródła podają 6 pasm o różnych nazwach):

| Pasmo | Kanały w zakresie (MHz) | Na krawędzi — unikaj | Poza zakresem |
|---|---|---|---|
| A | A2 5845, A3 5825, A4 5805, A5 5785, A6 5765, A7 5745 | A1 5865, A8 5725 | — |
| B | B2 5752, B3 5771, B4 5790, B5 5809, B6 5828, B7 5847 | B1 5733, B8 5866 | — |
| E | — | — | wszystkie (5645–5705, 5885–5945) |
| F | F1 5740, F2 5760, F3 5780, F4 5800, F5 5820, F6 5840, F7 5860 | — | F8 5880 |
| R (Raceband) | R4 5769, R5 5806, R6 5843 | R3 5732 | R1, R2, R7, R8 |
| 6. pasmo (L/„O”) | — | — | ⚠️ według instrukcji; pasmo „L” (5362–5621) jest całe poza zakresem |

Zalecenie: **F4 (5800 MHz)** lub **R5 (5806 MHz)** — środek zakresu. Przed lotem włącz
skanowanie w VR03 bez drona: jeśli na kanale jest Wi-Fi (część routerów 5 GHz używa
5745–5825 MHz), wybierz inny. Moc 25 mW (bez trybu wyższej mocy).

## 8. Lista elementów (BOM)

| # | Element | Ilość | Masa (szac.) | Uwagi |
|---|---|---|---|---|
| 1 | Eachine TX06 (wersja AIO 48CH, NTSC) | 1 | 2,8 g | bez uchwytu 26 mm |
| 2 | Dławik 4,7 µH, Isat ≥ 0,8 A, DCR ≤ 0,15 Ω, SMD 2520 (1008) | 1 | 0,03 g | |
| 3 | Kondensator 47 µF X5R ≥ 6,3 V, 0805/1206 | 1 | 0,03 g | |
| 4 | Kondensator tantalowy 22 µF 6,3 V, case A | 1 | 0,03 g | tłumienie rezonansu |
| 5 | Kondensator 100 nF X7R 0402/0603 | 1 | <0,01 g | |
| 6 | Przewód silikonowy 30 AWG, czerwony + czarny | 2 × 8 cm | 0,15 g | skręcone |
| 7 | Koszulka termokurczliwa 1,5–3 mm, taśma Kapton | — | 0,05 g | |
| 8 | Uchwyt z TPU 95A | 1 | ≤ 0,8 g | wydruk |
| 9 | (opcjonalnie) tranzystor do wyłączania kamery — patrz 11.3 | 1 | 0,02 g | |
| | **Razem** | | **≈ 3,9 g** | |

Narzędzia: lutownica z cienkim grotem, topnik, multimetr, zasilacz laboratoryjny z limitem
prądu (do testów kamery), waga 0,01 g.

## 9. Kroki montażu

1. **Test kamery na stole (zasilacz, nie bateria):** 3,7 V, limit 0,4 A. Sprawdź kolory
   przewodów (sekcja 1), obraz w VR03, ustaw kanał F4/R5. Zmierz prąd przy 3,3 / 3,7 / 4,2 V
   i napięcie, przy którym obraz znika. Po 3 min dotknij nadajnika — zanotuj, czy parzy.
2. **Test masy na uwięzi (bez lutowania):** przyklej taśmą 4 g w miejscu kamery. Zawis na
   uwięzi w Auto Alt, zapisz log. Sprawdź `thrust_duty` i `vbat_v` (oczekiwane: duty ok. +5%).
3. **Uchwyt:** zaprojektuj i wydrukuj, przymierz bez kamery (nie zasłania ToF, nie zwisa).
4. **Filtr:** zlutuj L1, C1–C3 na końcu przewodów przy kamerze, zaizoluj.
5. **Punkt zasilania:** wyjmij baterię. Znajdź `VBAT_IN` i GND (sekcja 2), przylutuj
   przewody, odprowadź je górą, przyklej Kaptonem z dala od śmigieł.
6. **Kontrola przed baterią:** omomierz między + a GND kamery — nie może być zwarcia
   (będzie widać ładowanie kondensatorów).
7. **Pierwsze włączenie:** bateria, obraz w goglach, dron w PARKING. Sprawdź, czy dron
   startuje normalnie (brak resetów ESP32 — LED, komunikaty w `pio device monitor`).
8. **Silniki bez śmigieł** (z pilota, na stole): czy w obrazie są pasy zmieniające się z gazem.
9. **Uwięź ze śmigłami**, potem niski lot w Position Hold. Logi: `vbat_v`, `thrust_duty`,
   ewentualnie `ibat_a` (11.1). Wyniki zapisz w `07-strojenie.md`.

## 10. Ryzyka

| Ryzyko | Skutek | Środki zaradcze |
|---|---|---|
| **Zwarcie na `VBAT_IN`** (brak bezpiecznika) | stopienie przewodu, uszkodzenie baterii, pożar | izolacja koszulką, prowadzenie z dala od śmigieł, krok 6; nie lutować z włożoną baterią |
| **Pomyłka masy z pinem 2 złącza silnika** | kamera zasilana przez tranzystor silnika, dziwne zachowanie, uszkodzenie | masa tylko z Grove pin 4 lub złącza baterii |
| **Zanik obrazu przy niskim napięciu** (min. 3,3 V vs próg lądowania 3,35 V) | utrata podglądu tuż przed lądowaniem | lądowanie w zasięgu wzroku; zmierzyć realny próg kamery (krok 1); ewentualnie podnieść `kBatteryLandVolt` o 0,05 V przy lotach FPV |
| **Reset ESP32 (brown-out)** | silniki stop w powietrzu | nie zasilać z 3,3 V ani z Grove; dławik ogranicza udar przy włączeniu; obserwacja w kroku 7 |
| **Przegrzanie nadajnika** | uszkodzenie kamery, deformacja uchwytu TPU | nie zostawiać włączonej na ziemi długo (lub wyłącznik 11.3); szczelina powietrzna w uchwycie |
| **Zakłócenia od silników** | pasy w obrazie | filtr LC, skręcone przewody, krok 8 |
| **Zakłócenie łączności ESP-NOW (2,4 GHz) przez nadajnik 5,8 GHz** | utrata łączności → failsafe | inne pasmo, ale harmoniczne/odbiór blisko: antena VTX daleko od StampS3; obserwuj `rc_age_loops` w logu |
| **Zakłócenie BMM150** | brak (magnetometr nieużywany) | — |
| **Zbyt mały zapas ciągu przy słabej baterii** | wolniejsze wznoszenie, gorsze trzymanie wysokości | test masy (krok 2); propozycja 11.2 |
| **Przesunięty środek ciężkości** | ciągły przechył, gorszy Position Hold | kamera blisko środka, korekta położenia baterii |
| **Zasłonięcie ToF lub PMW3901** | błędna wysokość / brak Position Hold (fallback) | sekcja 6; po montażu sprawdź `tof_raw_mm`, `flow_squal` |

## 11. Propozycje zmian w firmware (diff)

**Nie zastosowane.** Każda wymaga Twojej zgody; wszystkie w stylu projektu (haczyki
`[POSHOLD]` w plikach upstreamu, logika w naszych plikach).

### 11.1 Prąd baterii w telemetrii (`ibat_a`)

Pozwala zmierzyć prąd zawisu (I₀ w sekcji 4) i pobór kamery. Odczyt co 8. obieg (50 Hz),
żeby nie dokładać transakcji I2C w każdym obiegu 400 Hz.

```diff
--- a/drone/src/sensor.cpp
+++ b/drone/src/sensor.cpp
@@
 volatile float Voltage;
+volatile float Battery_current = 0.0f;  // [POSHOLD] INA3221 CH2, shunt R12 = 10 mOhm (library default) [A]
@@
     // Battery voltage check
     Voltage   = ina3221.getVoltage(INA3221_CH2);
+    // [POSHOLD] whole-drone current through the 10 mOhm shunt, 50 Hz is enough for logs
+    static uint8_t current_div = 0;
+    if (++current_div >= 8) {
+        current_div     = 0;
+        Battery_current = ina3221.getCurrent(INA3221_CH2);
+    }
     filterd_v = voltage_filter.update(Voltage, Control_period);
--- a/drone/src/sensor.hpp
+++ b/drone/src/sensor.hpp
@@
 extern volatile uint8_t Under_voltage_flag;
+extern volatile float Battery_current;  // [POSHOLD]
--- a/drone/src/telemetry_stream.cpp
+++ b/drone/src/telemetry_stream.cpp
@@
     {"vbat_v", []() -> float { return (float)Voltage; }},
+    {"ibat_a", []() -> float { return (float)Battery_current; }},
```

Do sprawdzenia przy wdrożeniu: czas odczytu I2C (`loop_dt_s` musi zostać ≈ 0,0025 s),
liczba pól telemetrii (limit `kMaxDataFields`), znak prądu (oczekiwany dodatni przy
rozładowaniu: IN+2 po stronie baterii).

### 11.2 Korekta trymu zawisu o masę ładunku

```diff
--- a/drone/lib/poshold/src/poshold_config.hpp
+++ b/drone/lib/poshold/src/poshold_config.hpp
@@
+// Extra mass carried (e.g. FPV camera + mount) relative to the stock drone with battery.
+// The hover trim scales with sqrt(m / m0) (thrust ~ (duty * V)^2). 0 = stock drone.
+constexpr float kPayloadMassG = 0.0f;   // [g] e.g. 3.9 for the TX06 camera kit
+constexpr float kStockMassG   = 36.8f;  // [g] drone with battery, CALIBRATE
--- a/drone/src/flight_control.cpp
+++ b/drone/src/flight_control.cpp
@@
 #include "alerts.hpp"             // [POSHOLD]
+#include <poshold_config.hpp>     // [POSHOLD]
@@
 float get_trim_duty(float voltage) {
-    return -0.2448f * voltage + 1.5892f;
+    // [POSHOLD] scale the upstream trim for an extra payload (poshold_config.hpp)
+    static const float payload_scale =
+        sqrtf((poshold::config::kStockMassG + poshold::config::kPayloadMassG) / poshold::config::kStockMassG);
+    return (-0.2448f * voltage + 1.5892f) * payload_scale;
 }
```

Najpierw sprawdź w logu z próby masy (krok 9.2), czy `thrust_duty − thrust0` faktycznie
ustawia się na ok. +5%; jeśli tak, ta zmiana przywraca pełny zakres ±15% regulatora wysokości.

### 11.3 (opcjonalnie) Wyłączanie kamery na ziemi

Wymaga dodatkowego tranzystora (np. P-MOSFET po stronie plusa sterowany przez tranzystor
NPN/N-MOSFET) z pinu Grove **G1 lub G2** (złącze czarne, DOC). Firmware: kamera włączona
w FLIGHT/AUTO_LANDING i przez 30 s po wylądowaniu, wyłączona w PARKING. Zmniejsza
nagrzewanie i pobór na ziemi. Diff zaproponuję po wyborze tranzystora i pinu, bo wymaga
decyzji sprzętowej.

### 11.4 Progi baterii

**Bez zmian** do czasu logów z kamerą. Jeśli obraz gaśnie przed lądowaniem, podnieś
`kBatteryLandVolt` z 3,35 do 3,40 V (`poshold_config.hpp`).

## 12. Czego nie udało się ustalić

| Co | Dlaczego | Jak ustalić |
|---|---|---|
| Kolory i funkcje przewodów TX06 | brak oficjalnej instrukcji online (tylko agregator) | instrukcja z pudełka, multimetr (sekcja 1) |
| Pobór prądu TX06 w funkcji napięcia, realny próg zaniku obrazu | dane sprzedawcy bez warunków pomiaru | zasilacz laboratoryjny (krok 9.1) |
| Tabela kanałów TX06 (nazwy 6 pasm) | źródła niespójne (A, B, E, F, R + „O”/L) | instrukcja z pudełka, skanowanie w VR03 |
| Rozmieszczenie elementów na PCB (gdzie fizycznie jest `VBAT_IN`, C22/C23) | schemat nie pokazuje layoutu | multimetr (sekcja 2) |
| Typ i wydajność regulatora 3,3 V w StampS3 | jest w module, nie na schemacie płyty | dokumentacja StampS3 — i tak odradzamy tę opcję |
| Rezystancja wewnętrzna baterii | brak danych | z logu: Δ`vbat_v` / Δ`ibat_a` przy zmianie gazu (po 11.1) |
| Prąd zawisu | firmware go nie loguje | propozycja 11.1 |
| Dokładne wymiary mocowań ramy | rysunek DOC ma tylko gabaryty | suwmiarka |

## Źródła

- Dokumentacja StampFly (M5Stack): <https://docs.m5stack.com/en/app/StampFly> oraz PDF tej
  strony z wbudowanymi stronami schematów (`Stamp_Fly_v1.0`, płytka przednia, PMW3901):
  <https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/static/pdf/static/en/app/StampFly.pdf>.
  Osobne pliki `Stamp_Fly_v1.0.pdf` i `StampS3_Fly_Hat.pdf` (aliyuncs `m5stack-doc`)
  nie były dostępne z sieci środowiska, w którym powstała analiza.
- Kod: `drone/src/sensor.cpp`, `drone/src/flight_control.cpp`, `drone/src/flight_control.hpp`,
  `drone/src/led.cpp`, `drone/lib/poshold/src/poshold_config.hpp`, biblioteka INA3221
  (`drone/.pio/libdeps/.../INA3221/src/INA3221.cpp`: domyślny bocznik 10 mΩ, `getCurrent()`).
- Eachine TX06 (oferta): <https://www.banggood.com/Eachine-TX06-700TVL-FOV-120-Degree-5_8Ghz-48CH-Smart-Audio-Mini-FPV-Camera-Support-Pitmode-AIO-Transmitter-For-RC-Drone-Tiny-Whoop-p-1413572.html>;
  opis przewodów (niepotwierdzony): <https://manuals.plus/ae/1005007187471802>;
  forum: <https://forum.flitetest.com/index.php?threads/eachine-tx06.64125/>.
- BETAFPV VR03: <https://betafpv.com/products/vr03-fpv-goggles>.
- Zakres 5725–5875 MHz, 25 mW e.i.r.p.: CEPT ERC Recommendation 70-03, załącznik 1
  (urządzenia SRD ogólnego przeznaczenia).
