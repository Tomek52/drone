# ADR 0005: Protokół ESP-NOW v2 z polem wersji, wybór trybu i kill switch

- Status: przyjęta
- Data: 2026-10-14
- Gałąź: `feat/joystick-mode`

## Kontekst

- Nowy tryb lotu trzeba przekazać z pilota do drona. Pakiet v1 (25 B) nie ma wolnych
  pól ani numeru wersji, a dron nie sprawdza jego długości.
- Niezgodne firmware'y (fabryczny pilot + nasz dron lub odwrotnie) nie mogą doprowadzić
  do startu z błędnie interpretowanymi danymi.
- Wymaganie: natychmiastowe wyłączenie silników z pilota w każdym trybie. W upstreamie
  wyłączenie wymaga dwóch kroków (auto-lądowanie, potem ponowne wciśnięcie).
- Firmware STM32 w pilocie nie jest modyfikowany — dostępne są tylko 2 przyciski przednie,
  2 wciśnięcia drążków i ekran AtomS3. Wciśnięcie lewego drążka nie jest w upstreamie używane.

## Decyzja

1. **Pakiet v2 = v1 + 4 bajty** (wersja, tryb lotu, flagi, zarezerwowany) + suma kontrolna
   na końcu. Bajty 0–23 bez zmian — upstreamowy kod parsowania drążków działa dalej.
2. **Dron odrzuca pakiety o innej wersji/długości** i nie aktualizuje drążków — przycisk
   startu nie dociera, więc start jest niemożliwy. Sygnalizacja: LED czerwony/niebieski,
   buzzer.
3. **Pakiet statusu dron → pilot** (8 B, 5 Hz): wersja, faktycznie latany tryb, przyczyna
   fallbacku. Pilot rozpoznaje stary firmware drona po braku statusu przy obecnej telemetrii.
4. **W Position Hold bajt 21 = 0 (Stable)** — stary dron, gdyby jednak przyjął pakiet,
   poleci w Stable, nigdy w nieznanym trybie.
5. **Kill switch: przytrzymanie lewego drążka 0,5 s** — flaga w każdym pakiecie przez cały czas
   trzymania + 0,5 s po puszczeniu. Dron w trybach FLIGHT/FLIP/AUTO_LANDING natychmiast
   zatrzymuje silniki i przechodzi do PARKING.
6. **Zmiana trybu tylko na ziemi**, w locie jedynie Position Hold → Stable.
7. Dźwięki pilota są odtwarzane w wątku LVGL — `beep()` blokuje wątek, a blokada głównej
   pętli > 100 ms wywołałaby u drona failsafe utraty łączności.

## Rozważane alternatywy

| Alternatywa | Dlaczego nie |
|---|---|
| Nowy układ pakietu od zera | Więcej zmian w kodzie upstreamu, trudniejsze `subtree pull`. |
| Kodowanie trybu w nieużywanych bitach v1 | Brak miejsca na wersję; stary dron zinterpretowałby nieznane wartości. |
| Kill na przycisku ekranu (długie wciśnięcie) | Ekran = start/lądowanie; ryzyko pomyłki. Wciśnięcie drążka jest wolne i dostępne bez odrywania kciuka. |
| Kill po krótkim wciśnięciu | Ryzyko przypadkowego wyłączenia w locie przy energicznym ruchu drążkiem. |
| Zmiana trybu w locie bez ograniczeń | Przypadkowe wejście w Sport w powietrzu jest groźne. |

## Konsekwencje

- Dron i pilot muszą mieć firmware z tej samej wersji protokołu; fabryczne firmware'y
  nie współpracują z naszymi (bezpiecznie — brak startu).
- Na ekranie lotu tryb zastępuje odczyt przedniego ToF.
- Drugie wciśnięcie lewego przycisku w locie zachowuje się jak w upstreamie
  (powrót do ekranu startowego + lądowanie).
