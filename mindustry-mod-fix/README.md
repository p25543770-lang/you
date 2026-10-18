# Исправление «Campaign Threat Radar» для последней версии Mindustry

На скриншоте — менеджер модов Mindustry (v8, build 160.7). Ошибка
«Несовместимый» возникает потому, что в `mod.json` мода **Campaign Threat Radar**
отсутствует поле `minGameVersion`. Mindustry требует `minGameVersion >= 136`
(для v8), а по подсказке в игре минимум — **154**.

В этой папке:

- `patch-mod.sh` — патчит установленный мод на вашем диске (Windows / Linux /
  Android — выберите нужную папку модов, см. ниже).
- `mod.json` — готовый исправленный шаблон `mod.json` с
  `"minGameVersion": "154"`.

## Куда класть / где искать мод «Campaign Threat Radar»

| ОС        | Путь к папке модов                                                  |
|-----------|---------------------------------------------------------------------|
| Windows   | `%APPDATA%\Mindustry\mods\`                                         |
| Linux     | `~/.local/share/Mindustry/mods/`                                    |
| macOS     | `~/Library/Application Support/Mindustry/mods/`                     |
| Android   | внутренний накопитель → `Mindustry/mods/`                           |
| Steam     | `<Steam>/steamapps/common/Mindustry/mods/` (редко)                  |

Ищите папку (или `.zip`) с именем вроде `campaign-threat-radar` или похожим,
внутри — файл `mod.json` (или `mod.hjson`).

## Быстрый ручной фик

1. Откройте `mod.json` (или `mod.hjson`) мода любым текстовым редактором
   (Notepad, VS Code, …).
2. Найдите секцию с `name`, `displayName`, `author`, `version`, …
3. Добавьте строку (запятую в конце предыдущей строки не забудьте):

       "minGameVersion": "154",

   Для `mod.hjson` (без кавычек у ключей):

       minGameVersion: "154"

4. Сохраните файл и перезапустите игру. Мод перестанет быть «Несовместимым».

Если мод упакован в `.zip`, распакуйте его в подпапку `mods/`, исправьте
`mod.json` внутри папки и перезапустите игру (либо перепакуйте в `.zip`).
