# core/neuro_controller/llm_bridge.py
import openai
from ollama import generate

class LLMController:
    def __init__(self):
        self.openai_key = "your-api-key"
        self.local_model = "quantized_mistral"  # Для RPi

    def process_command(self, text):
        if is_complex_task(text):  # Сложные задачи -> OpenAI
            response = openai.ChatCompletion.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": f"Как роботу выполнить: {text}?"}]
            )
            return response.choices[0].message.content
        else:  # Простые задачи -> локальная Mistral
            return generate(model=self.local_model, prompt=text)
