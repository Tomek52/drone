# ADR 0004: Filtr komplementarny (z estymacją biasu) zamiast filtru Kalmana

- Status: przyjęta
- Data: 2026-10-14
- Gałąź: `feat/position-estimator`

## Kontekst

Prędkość poziomą trzeba wyznaczyć z dwóch źródeł:

- optical flow (PMW3901, 100 Hz): bez dryfu, ale zaszumiony i mocno skwantowany
  (≈ 1 m/s na zliczenie na próbkę na wysokości 0,5 m), z przerwami (zła tekstura);
- akcelerometr (400 Hz): gładki, ale całkowanie dryfuje (bias, błąd orientacji).

Wymaganie: prosta, przewidywalna implementacja na ESP32 w pętli 400 Hz, łatwa
do strojenia przez użytkownika na podstawie logów.

## Decyzja

Dla każdej osi (x, y w układzie heading) dwustanowy filtr komplementarny:
predykcja `v += (a − b)·dt`, korekta `v += k·e`, `b −= k_b·e`, gdzie
`e = v_flow − v`. Parametry: dwie stałe czasowe (`kVelCorrectionTau`,
`kAccelBiasTau`) w `poshold_config.hpp`. Osie są niezależne.

## Rozważane alternatywy

| Alternatywa | Dlaczego nie (na razie) |
|---|---|
| Rozszerzony filtr Kalmana (pozycja, prędkość, bias, wysokość, orientacja) | Lepszy teoretycznie, ale trudny do strojenia (macierze Q/R), więcej obliczeń, łatwo o błąd; upstream ma już osobne estymatory orientacji i wysokości. |
| Liniowy Kalman 2-stanowy na oś | Matematycznie równoważny filtrowi komplementarnemu w stanie ustalonym; strojenie przez wariancje jest mniej intuicyjne niż stałe czasowe. |
| Tylko optical flow (bez akcelerometru) | Zbyt zaszumione do regulatora prędkości; brak „przetrwania” krótkich przerw flow. |

## Konsekwencje

- Dwie intuicyjne stałe czasowe do strojenia; zachowanie sprawdzone w testach
  `native` (zbieżność, kompensacja obrotu, estymacja biasu, przerwy flow).
- Brak formalnej niepewności estymaty — jakość opisują bramki (`flow_valid`,
  `flow_reject`, SQUAL).
- W przyszłości można wymienić estymator na Kalmana bez zmiany interfejsu
  `PositionEstimator`.
