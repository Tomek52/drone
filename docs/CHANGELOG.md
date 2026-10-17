# Changelog

Zmiany według gałęzi / PR (najnowsze na górze).

## `chore/repo-setup`

- Przypięte wersje platformy i bibliotek PlatformIO w obu projektach (ADR 0002).
- LVGL 8.3.10 dla pilota pobierany z rejestru PlatformIO zamiast archiwum ZIP z GitHuba.
- Środowisko `native` z Unity i pierwszym testem (`test_math_utils`), biblioteka `drone/lib/poshold`.
- GitHub Actions: kompilacja drona i pilota, testy `native`.
- Szkielet dokumentacji `docs/`, ADR 0001 (struktura repozytorium), ADR 0002 (toolchain).
- Plik `stampfly.code-workspace` dla VS Code, główny `README.md`, `.gitignore`.

## `main` (import)

- Import `m5stack/M5StampFly@fa7b6a5` do `drone/` (git subtree, pełna historia).
- Import `m5stack/Atom-JoyStick@e9f53fb` (`examples/StampFlyController`) do `joystick/`.
- Bez zmian w kodzie upstreamu.
