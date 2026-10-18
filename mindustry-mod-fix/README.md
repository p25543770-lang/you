# Как починить «Campaign Threat Radar» — 3 шага вручную

На скриншоте ошибка: **«Этот мод несовместим с последней версией игры. Автор должен
обновить его и добавить minGameVersion: 154 в файл mod.json.»**

Речь про мод **Campaign Threat Radar** (версия 1, автор — `local`, т.е. вы
импортировали его вручную кнопкой «Импортировать модификацию»).
Чтобы он заработал, нужно ОДИН раз вручную дописать одну строку в его
`mod.json` / `mod.hjson`.

## Шаг 1 — Найдите папку Mindustry с модами

| Система | Путь |
|---------|------|
| **Windows** | Откройте Проводник, в адресной строке наберите `%APPDATA%\Mindustry\mods\` и нажмите Enter. |
| **Linux** | `~/.local/share/Mindustry/mods/` |
| **macOS** | `~/Library/Application Support/Mindustry/mods/` |
| **Android** | Через файловый менеджер: внутренняя память → `Mindustry/mods/` (если нет — ищите в `Android/data/io.anuke.mindustry/files/mods/`, нужен доступ к файлам). |

Импортированные локально моды лежат прямо в этой папке в виде **подпапки**
(обычно имя совпадает с названием мода, или имеет вид
`1728394050000_Campaign Threat Radar`), либо как `.zip`.

## Шаг 2 — Откройте `mod.json` (или `mod.hjson`)

Внутри найденной папки (или внутри `.zip`, который придётся распаковать)
есть файл `mod.json` или `mod.hjson`. Откройте его Блокнотом / VS Code /
любым текстовым редактором.

Содержимое будет выглядеть примерно так:

```json
{
  "name": "campaign-threat-radar",
  "displayName": "Campaign Threat Radar",
  "author": "local",
  "version": "1",
  "description": "Client-only campaign HUD showing enemy units and likely approach paths..."
}
```

(Если у вас `mod.hjson` — без фигурных скобок и кавычек у ключей — читайте
конец инструкции.)

## Шаг 3 — Добавьте строку `minGameVersion`

Добавьте **одну строку** (не забудьте запятую в конце предыдущей строки):

```json
"minGameVersion": "154",
```

Итог должен выглядеть так:

```json
{
  "name": "campaign-threat-radar",
  "displayName": "Campaign Threat Radar",
  "author": "local",
  "version": "1",
  "minGameVersion": "154",
  "description": "Client-only campaign HUD showing enemy units and likely approach paths..."
}
```

Сохраните файл. Если вы правили `.zip` — перепакуйте содержимое обратно в
zip (убедитесь, что `mod.json` лежит в **корне** архива, а не во вложенной
папке), и положите в `mods/`.

✅ **Полностью закройте игру и запустите заново** (просто перезагрузить список
модов недостаточно — Mindustry читает манифесты только при старте).
После этого вместо «Несовместимый / Выключено» мод будет доступен для включения.

---

### Если у вас `mod.hjson` (формат с минимумом кавычек)

Там всё то же самое, но ключ без кавычек:

```hjson
name: campaign-threat-radar
displayName: Campaign Threat Radar
author: local
version: "1"
minGameVersion: "154"
description: Client-only campaign HUD showing enemy units and likely approach paths
```

### Какое число ставить?

- `154` — **минимальное значение**, которое требует сама игра на скриншоте
  (подсказка «добавьте minGameVersion: 154»). С ним мод заведётся на v8.
- Если у вас самая свежая Mindustry build **160.7** (на октябрь 2026) — можно
  поставить `"160.7"`, но `"154"` тоже работает, потому что это только
  *минимальная* требуемая версия.
