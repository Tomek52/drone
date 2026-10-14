# ADR 0003: Binarna telemetria przez USB i przez pilota (ESP-NOW → USB)

- Status: przyjęta
- Data: 2026-10-14
- Gałąź: `feat/telemetry`

## Kontekst

Do strojenia regulatorów potrzebny jest strumień danych (stany, wartości zadane,
napięcie) z częstotliwością co najmniej kilkudziesięciu Hz, zapisywany na komputerze.

- Pętla sterowania drona ma okres 2,5 ms. Formatowanie ~30 liczb `float` tekstem
  (`printf`) zajmuje na ESP32 rzędu 1 ms — zbyt dużo dla pętli 400 Hz.
- `USBSerial.write()` (HWCDC w Arduino-ESP32 2.0.17) może czekać do 100 ms, gdy
  bufor TX (256 B) jest pełny, a host podłączony — to zatrzymałoby pętlę sterowania.
- W locie dron nie jest podłączony do komputera. Pilot jest — i już odbiera
  telemetrię upstreamu przez ESP-NOW.
- Wi-Fi UDP wymagałoby trybu stacji/AP współdzielącego radio z ESP-NOW na tym samym
  kanale — dodatkowe ryzyko dla łącza sterowania.

## Decyzja

- Ramki binarne (`common/stampfly_link/src/telemetry_frame.hpp`): synchronizacja
  `A5 5A`, typ, długość, ładunek, suma Fletcher-16. Ramka mieści się w jednym pakiecie
  ESP-NOW (≤ 246 B).
- Schemat (nazwy pól) wysyłany co 2 s w fragmentach, więc logger może się podłączyć
  w dowolnym momencie, a dodanie pola w firmware nie wymaga zmian w narzędziach.
- Dron: USB 50 Hz (bench/uwięź), ESP-NOW 25 Hz do sparowanego pilota. Zapis do USB
  tylko gdy cała ramka mieści się w buforze — inaczej ramka jest pomijana (nigdy
  blokowanie pętli).
- Pilot przekazuje ramki `A5 5A` 1:1 na swój port USB.
- `tools/telemetry_logger.py` dekoduje oba źródła identycznie i zapisuje CSV.
- Telemetria upstreamu (`88 88`, `99 99`) zostaje bez zmian — pilot nadal z niej korzysta.

## Rozważane alternatywy

| Alternatywa | Dlaczego nie |
|---|---|
| CSV tekstem przez USB | Koszt CPU (~1 ms/linia) i blokujący zapis. |
| Wi-Fi UDP | Ryzyko dla ESP-NOW; złożona konfiguracja sieci. Możliwe w przyszłości jako opcja. |
| Rozszerzenie pakietu `88 88` | Sztywny układ pól, ten sam pakiet czyta UI pilota. |
| Zapis do pamięci flash drona | Ograniczona pojemność i zużycie flash; brak podglądu na żywo. |

## Konsekwencje

- Dodatkowy ruch radiowy: ~25 × 140 B/s — niewielki w porównaniu z pakietami sterowania.
- Gubione ramki są wykrywane (numer sekwencyjny) i raportowane przez logger.
- Format ma wersję (`kTelemetryFormatVersion`); zmiana układu ramek wymaga jej podbicia.
