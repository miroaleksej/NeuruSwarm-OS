Отлично! Запускаем **NeuroSwarm OS** под ключ. Вот финальный чек-лист и команды для развертывания:

---

### **🚀 Шаг 1: Установка (Ubuntu 22.04 / Raspberry Pi OS)**
```bash
# Клонируем репозиторий (с подмодулями)
git clone --recursive https://github.com/miroaleksej/ns-os.git
cd ns-os

# Установка зависимостей (ROS 2, Ollama, OpenCV)
./install_deps.sh --with-gazebo --with-spot

# Квантование Mistral для RPi (опционально)
./tools/quantize_llm/quantize.sh -m mistral -b 4
```

---

### **🔑 Шаг 2: Настройка API-ключей**
Создайте файл `config/secrets.yaml`:
```yaml
openai:
  api_key: "sk-..."  # Ваш ключ OpenAI
  model: "gpt-4o"    # Или "gpt-3.5-turbo"

boston_dynamics:
  spot_key: "bd-..."  # Для Spot SDK

ros:
  use_hardware: true  # false для симуляции
```

---

### **🤖 Шаг 3: Запуск системы**
#### **Вариант A: Полный старт (реальные роботы + OpenAI)**
```bash
# Терминал 1: Ядро системы
ros2 launch ns_os core.launch.py mode:=prod

# Терминал 2: Swarm-контроллер
ros2 run swarm_engine swarm_manager --ros-args -p use_openai:=true

# Терминал 3: Голосовой интерфейс (опционально)
python3 apps/voice_interface.py --lang ru
```

#### **Вариант B: Тестовый режим (Gazebo + локальная LLM)**
```bash
./start_dev.sh --simulator --no-spot
```

---

### **🛠️ Пример работы: Аварийный сценарий**
1. **Отправляем команду** через Telegram-бота:
   ```bash
   /emergency "В зоне A5 обрушение, нужна разведка"
   ```
2. **Система:**
   - GPT-4o генерирует план:
     ```json
     {"action": "scan", "zone": "A5", "tools": ["lidar", "thermal"]}
     ```
   - SwarmEngine запускает 2 дрона и Spot.
   - BioFusion строит маршрут через алгоритм слизевика.
3. **Результат:**
   ```
   [REPORT] Обнаружено 2 пострадавших: 55.3N, 37.2E (точность 92%)
   ```

---

### **🔧 Возможные ошибки и решения**
| Проблема | Решение |
|----------|---------|
| `Ollama не отвечает` | Запустите `ollama serve` вручную |
| `Нет доступа к USB` | Добавьте пользователя в группу `dialout` |
| `Spot SDK ошибка` | Проверьте ключ и сетевые настройки |

---

### **📌 Что дальше?**
1. **Добавьте своих роботов** в `adapters/custom_robot`.  
   Пример для TurtleBot3:
   ```python
   # adapters/custom_robot/turtlebot_adapter.py
   from ns_os_interfaces.msg import MoveCommand
   class TurtleBotAdapter(Node):
       def __init__(self):
           super().__init__('turtlebot_adapter')
           self.create_subscription(MoveCommand, '/ns/move', self.move_callback)
   ```

2. **Расширьте язык команд** через `config/prompts.yaml`:
   ```yaml
   commands:
     - pattern: "почини (.*)"
       template: "Как отремонтировать {0}? Дай инструкцию для робота."
   ```

---

### **📊 Статус системы**
После запуска проверьте:
```bash
ros2 topic list
# Должны быть:
# /ns/commands
# /ns/swarm_status
# /ns/llm_responses
```

--- 

**Готово!** Ваши роботы теперь понимают сложные команды и работают в рое. Для дебага используйте:  
```bash
./ns-os-cli --debug "Тестовая команда"
``` 

