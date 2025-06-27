#!/usr/bin/env python3
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch
import os

def quantize_model(model_name, output_dir, bits=4):
    """Quantize a language model to specified bit precision"""
    try:
        # Load model and tokenizer
        model = AutoModelForCausalLM.from_pretrained(model_name)
        tokenizer = AutoTokenizer.from_pretrained(model_name)
        
        # Quantize the model
        quantized_model = torch.quantization.quantize_dynamic(
            model,
            {torch.nn.Linear},
            dtype=torch.qint8 if bits == 8 else torch.quint4x2
        )
        
        # Save quantized model
        os.makedirs(output_dir, exist_ok=True)
        quantized_model.save_pretrained(output_dir)
        tokenizer.save_pretrained(output_dir)
        
        print(f"Successfully quantized {model_name} to {bits}-bit and saved to {output_dir}")
        return True
        
    except Exception as e:
        print(f"Error quantizing model: {str(e)}")
        return False

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='mistralai/Mistral-7B')
    parser.add_argument('--output', type=str, default='./quantized_models')
    parser.add_argument('--bits', type=int, choices=[4, 8], default=4)
    args = parser.parse_args()
    
    quantize_model(args.model, args.output, args.bits)
