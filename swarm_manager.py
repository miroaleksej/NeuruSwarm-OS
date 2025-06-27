# core/swarm_engine/swarm_manager.py
def assign_task(task):
    if task["priority"] > 0.8:  # Срочная задача -> реальные роботы
        spot_adapter.send_command(task)
    else:  # Тестирование в симуляторе
        gazebo_simulator.spawn_agents(task)
