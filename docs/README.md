# Dokumentacja projektu StampFly Position Hold

Dokumentacja jest po polsku, a kod i komentarze w kodzie po angielsku.
Informacje, których nie udało się potwierdzić w oficjalnych źródłach M5Stack,
są oznaczone: **⚠️ do weryfikacji na sprzęcie**.

| Dokument | Zawartość |
|---|---|
| [01-sprzet.md](01-sprzet.md) | Wersje sprzętu, czujniki, piny, magistrale, adresy I2C — ze źródłami |
| [02-instalacja-i-wgrywanie.md](02-instalacja-i-wgrywanie.md) | Środowisko (Windows 11), budowanie, wgrywanie, parowanie, M5Burner, aktualizacja z upstreamu |
| [03-architektura.md](03-architektura.md) | Moduły firmware'u, pętle i ich częstotliwości, przepływ danych, struktura repozytorium |
| [04-protokol-espnow.md](04-protokol-espnow.md) | Format pakietów pilot ⇄ dron, wersjonowanie, failsafe |
| [05-tryby-lotu.md](05-tryby-lotu.md) | Tryby lotu, przyciski, kolory LED |
| [06-position-hold.md](06-position-hold.md) | Estymacja i sterowanie w trybie Position Hold: wzory, układy współrzędnych, założenia, ograniczenia |
| [07-strojenie.md](07-strojenie.md) | Strojenie regulatorów krok po kroku z użyciem logów |
| [08-testy.md](08-testy.md) | Testy `native`, procedury testów na sprzęcie |
| [09-rozwiazywanie-problemow.md](09-rozwiazywanie-problemow.md) | Typowe problemy i ich rozwiązania |
| [10-plany.md](10-plany.md) | Rzeczy na później (m.in. wysokość bezwzględna nad przeszkodami) i otwarte sprawy po testach na sprzęcie |
| [11-stable-abs.md](11-stable-abs.md) | Tryb Stable ABS: wysokość z barometru, model podmuchu, plan testów |
| [12-lagodny-regulator-wysokosci.md](12-lagodny-regulator-wysokosci.md) | Projekt i symulacja łagodnego regulatora wysokości na barometrze (lot powyżej zasięgu ToF) |
| [13-telemetria-telefon.md](13-telemetria-telefon.md) | Podgląd i zapis telemetrii na telefonie przez Wi-Fi pilota |
| [fpv_camera_integration.md](fpv_camera_integration.md) | Kamera FPV Eachine TX06: zasilanie ze schematu, filtr, bilans masy i energii, INA3221, mocowanie, kanały 5,8 GHz |
| [decyzje/](decyzje/) | Architecture Decision Records (ADR) — jedna decyzja = jeden plik |
| [CHANGELOG.md](CHANGELOG.md) | Zmiany według gałęzi / PR |

## Zasady

- Dokumentacja zmienia się w tej samej gałęzi i PR co kod, którego dotyczy.
- Każda istotna decyzja projektowa ma swój ADR w `decyzje/` (kontekst, decyzja,
  rozważane alternatywy, konsekwencje).
- Diagramy w Mermaid (renderują się bezpośrednio na GitHubie).

## Różnica względem proponowanej struktury

Struktura odpowiada propozycji 1:1. Jedyny dodatek to sekcja „Aktualizacja
z upstreamu” w `02-instalacja-i-wgrywanie.md` (opis `git subtree pull`), bo
dotyczy codziennej pracy z repozytorium, a nie samej architektury.
