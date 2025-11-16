import os

DATA_DIR = "data"

OPENAI_API_KEY = ""
WORDS_JSON_FILE = f"{DATA_DIR}/deu_news_10K_words_clean.json"
TEMPERATURE = 0.7

# Path to the downloaded Piper model
PIPER_MODEL_PATH = "/Users/administrateur/piper_voices/de_DE-thorsten-medium.onnx"

# Path to save the generated WAV file
OUTPUT_WAV_PATH = "/Users/administrateur/Desktop/ETH_master/BMW/Projects/German_learner/data/voices/test.wav"

# Synthesis parameters
VOLUME = 1.0           # normal volume
LENGTH_SCALE = 0.8     # normal speed
NOISE_SCALE = 1.0      # standard noise scale
NOISE_W_SCALE = 1.0    # standard noise w scale
NORMALIZE_AUDIO = True # whether to normalize audio

