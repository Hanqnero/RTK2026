# Isaac Lab and WheeledLab Reference

## WheeledLab (локальный клон)

WheeledLab клонируется рядом с RTK2026 и используется как пример того, как устроены сцены и тренировки в Isaac Lab:

- **Путь:** рядом с RTK2026 (например, `workspace/WheeledLab`, где `workspace/RTK/RTK2026` — наш репозиторий). Скрипты ищут его автоматически; можно переопределить через `-WheeledLabPath`.
- **Репозиторий:** https://github.com/UWRobotLearning/WheeledLab

### Как WheeledLab запускает симуляции

1. **Startup** (`source/wheeledlab_rl/wheeledlab_rl/startup.py`):
   - `args_cli.enable_cameras = True` до создания `AppLauncher` (чтобы был видим viewport);
   - `AppLauncher.add_app_launcher_args(parser)`, затем `app_launcher = AppLauncher(args_cli)`.
2. **Тренировка (headless):**
   ```bash
   python source/wheeledlab_rl/scripts/train_rl.py --headless -r RSS_DRIFT_CONFIG
   python source/wheeledlab_rl/scripts/train_rl.py --headless -r RSS_ELEV_CONFIG
   python source/wheeledlab_rl/scripts/train_rl.py --headless -r RSS_VISUAL_CONFIG
   ```
3. **Стек:** Isaac Lab v2.0.2, Isaac Sim 4.5.0, Python 3.10 (у них Linux; мы используем Windows + Isaac Lab, где возможно).

### Сцена RTK2026 vs WheeledLab

- **WheeledLab** использует только USD‑активы (например, `mushr_nano.usd`, `f1tenth.usd`), не URDF.
- **Наша сцена**:
  - **С роботом (по умолчанию):** `scripts/run_isaac_lab.bat` — пол, свет, дифф‑робот (URDF, коробка + два колеса) в `/World/RTK2026`. Джойнты: `left_wheel_joint`, `right_wheel_joint`.
  - **Минимальная:** добавить `--no_robot` в скрипт или запустить `python run_rtk2026_scene.py --no_robot` — только пол и свет.

### Робот в сцене RTK2026

- **Если есть ассеты WheeledLab:** сцена автоматически подхватывает MuSHR (USD) из `WheeledLab/source/wheeledlab_assets/data/Robots/UWRLL/mushr_nano_v2.usd`. Робот с подвеской и приводами. Если `data` пустая, нужен `git lfs pull` в клоне WheeledLab.
- **Если ассетов нет:** используется наш дифф‑привод (URDF) с `fix_base=true`, чтобы робот не переворачивался.
- **Минимальная сцена:** запуск с флагом `--no_robot`.

### WheeledLab Visual (робот + датчики + traversability)

Сцена **Visual**: MuSHR, камера на роботе и террейн с картой проходимости.

1. Убедиться, что WheeledLab склонирован и выполнен `git lfs pull` (для ассетов).
2. Из корня RTK2026: `.\scripts\run_wheeledlab_visual.bat` или `.\scripts\run_wheeledlab_visual.ps1`.
3. Скрипт запускает Isaac Lab и сцену `Isaac-MushrVisualRL-v0` с `num_envs=1`. После открытия окна нажать **Play**.
4. В сцене: MuSHR, TiledCamera, террейн с проходимостью (semantic/traversability).

Сценарий сцены: `isaac_lab/run_wheeledlab_visual_scene.py`.

### Проверка сцены RTK2026 с роботом

1. Из корня RTK2026 запустить `.\scripts\run_isaac_lab.bat` (или `run_isaac_lab.ps1`).
2. Дождаться открытия окна Isaac и сообщения в консоли (`MuSHR` или `diff-drive robot (URDF, fixed base)`).
3. Нажать **Play** — видны пол, свет и робот.
4. Для минимальной сцены: добавить `--no_robot` в вызов `isaaclab.bat -p ...`.

### Готовые дифф‑приводы в Isaac Sim

В Isaac Sim / Isaac Lab нет отдельного стандартного «диф‑робота» — в основном манипуляторы.  
Для RTK2026 используется свой URDF: `rtk2026_diff_drive.urdf` (база + два колеса, джойнты `left_wheel_joint`, `right_wheel_joint`). WheeledLab даёт MuSHR (USD), но наш RTK‑робот основан на URDF.

