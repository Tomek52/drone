# Telemetria na telefonie

Pilot (Atom JoyStick) potrafi otworzyć własną sieć Wi-Fi i pokazać na telefonie podgląd telemetrii z drona
na żywo, a jednocześnie zapisywać ją do pliku CSV w formacie `tools/telemetry_logger.py`. Dzięki temu w parku
nie jest potrzebny laptop, a `plot_log.py` czyta plik z telefonu bez zmian.

**Status:** zaimplementowane, zbudowane, dekoder strony sprawdzony testem (Chrome/Edge headless). **Nie
sprawdzone na sprzęcie z telefonem** i nie sprawdzony wpływ na łącze sterujące (zob. „Test do wykonania”).

## Użycie

1. Na ekranie ustawień pilota (przed lotem) przytrzymaj **przycisk ekranu przez 2 s**. Pilot piknie i otwiera
   sieć `StampFly-Pilot` (hasło `stampfly1`). Funkcja jest domyślnie wyłączona, więc zwykłe loty nie zmieniają się.
2. Połącz telefon z tą siecią (telefon powie, że nie ma internetu, zostań w sieci) i otwórz `http://192.168.4.1/`.
3. Strona pokazuje stan połączenia, kafelki (wysokość, napięcie, tryby, `hm_trust`, ToF, prędkość pionowa),
   wykres 30 s i tabelę wszystkich pól. Zapis rusza sam z pierwszą ramką.
4. Po locie **Zapisz CSV** pobiera plik `stampfly_RRRRMMDD_GGMMSS.csv` do telefonu. Przenieś go do `logs/`.
5. Wyłączenie: ponowne przytrzymanie przycisku ekranu 2 s restartuje pilota (Wi-Fi znika).

Ekran telefonu musi być włączony, a karta przeglądarki na wierzchu. Po wygaszeniu ekranu WebSocket się zrywa
i w pliku jest dziura (widać ją w `time_s` i `seq`). Ustaw najdłuższy czas wygaszania. Strona próbuje
też zablokować wygaszanie (`wakeLock`), ale ta funkcja przeglądarki wymaga HTTPS, więc na zwykłym HTTP z pilota
zwykle nie działa.

## Jak to działa

- `joystick/src/phone_telemetry.{h,cpp}`: tryb `WIFI_AP_STA`, access point na **tym samym kanale co łącze
  ESP-NOW z dronem** (radio jest jedno), serwer HTTP (port 80) i WebSocket (port 81, biblioteka
  `links2004/WebSockets` 2.6.1) w osobnym zadaniu na rdzeniu 0 o niskim priorytecie. Pętla sterowania pilota
  (rdzeń 1) nie czeka na telefon.
- Ramki telemetrii z `telemetry_forward()` trafiają do kolejki (16 ramek, bez blokowania; przy pełnej kolejce
  ramka jest gubiona) i są rozsyłane binarnie, jedna ramka na wiadomość.
- Strona: `joystick/web/telemetry.html` (dekoder i zapis w JavaScript, lustro `tools/stampfly_telemetry.py`),
  wbudowana w `joystick/src/phone_page.h`. Po zmianie strony: `py tools/gen_phone_page.py`.
- Strona **nie ma żadnej drogi do wysyłania czegokolwiek do drona**. Sterowanie i KILL zostają na pilocie.
- Telemetria po ESP-NOW ma 25 Hz (kabel USB: 50 Hz), osobny licznik `seq`.

## Ograniczenia i ryzyko

- Wi-Fi i ESP-NOW dzielą radio pilota. Ruch Wi-Fi (beacony, WebSocket) zabiera czas antenie i może
  opóźniać lub gubić pakiety sterujące. Dlatego funkcja jest opt-in i **przed użyciem w locie wymaga testu
  na biurku** (poniżej).
- Kanał sieci jest ustawiany przy włączeniu. Jeśli pilot zmieni kanał (parowanie), sieć nie podąża za nim.
  Włączaj Wi-Fi dopiero po sparowaniu i uruchomieniu pilota.
- Do 2 telefonów naraz. Pamięć strony rośnie ok. 10 kB/s zapisu (10 min to ok. 6 MB).
- Hasło jest w kodzie i w tym dokumencie (sieć nie niesie żadnych poleceń do drona).

## Test do wykonania (bez śmigieł)

1. Dron na biurku bez śmigieł, zasilany z baterii, pilot sparowany. Logger USB na pilocie (`--raw`) przez 60 s
   bez Wi-Fi: zapisz wiersze, `lost`, `bad`.
2. To samo przy włączonym Wi-Fi z podłączonym telefonem i otwartą stroną. Porównaj liczbę wierszy i `lost`
   (powinny się nie pogorszyć) oraz `rc_age_loops` w telemetrii drona (opóźnienie pakietów sterujących),
   z poruszaniem drążkami.
3. Dopiero po przejściu: lot testowy z włączoną stroną i zapisem w telefonie, jednocześnie logger na laptopie
   jako odniesienie (kabel USB daje 50 Hz).
