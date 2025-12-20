# German Sentence Generator with Speech Synthesis

This project is a web application that generates natural German sentences using a specified set of words, provides their English translations, and synthesizes German speech using a neural voice model. It is designed to assist language learners in practicing vocabulary and pronunciation.

A key feature is adaptive word learning: users can click on words they don’t understand, and these words will appear with a higher probability in subsequent rounds of sentence generation, allowing for personalized vocabulary practice.

---

## Core Modules and Implementation

### 1. `generation.py`
- **Purpose:** Generates natural German sentences and their English translations.
- **Key Function:** `generate_sentence(client, words, temperature=0.7)`
  - Accepts a list of words and produces a grammatically correct German sentence including all of them.
  - Uses the OpenAI `chat.completions.create` API (GPT-4.1) to generate sentences.
  - Ensures structured output with:
    ```
    GERMAN: <German sentence>
    ENGLISH: <English translation>
    ```
  - Handles parsing and validation of the model output.

---

### 2. `sampling.py`
- **Purpose:** Samples words from a frequency-based German vocabulary.
- **Key Function:** `sample_words(json_file, n)`
  - Reads a JSON file containing German words and associated probabilities.
  - Randomly samples `n` words according to their frequency distribution.
  - Supports adaptive learning by increasing the sampling probability of words that the user clicked as "unknown" in previous rounds.
  - Useful for generating sentences with realistic word usage and personalized learning.

---

### 3. `voicing/german_voice.py`
- **Purpose:** Converts German text into speech.
- **Key Function:** `synthesize_speech(text: str) -> str`
  - Uses a Piper voice model for text-to-speech synthesis.
  - Configurable parameters:
    - Volume
    - Length scale
    - Noise scale
    - Normalization
  - Saves the synthesized speech as a `.wav` file and returns the file path.
- **Output:** High-quality German audio ready for playback in the web app.

---

### 4. `app.py`
- **Purpose:** Flask web application serving the interactive interface.
- **Endpoints:**
  - `/`  
    Serves the main HTML page.
  - `/generate_german` [POST]  
    Generates a German sentence, its English translation, and the corresponding audio file.
  - `/audio/german_sentence.wav`  
    Serves the generated audio file.
  - `/get_english` [POST]  
    Returns the English translation of a given German sentence.
- **Integration:** Combines all modules (`generation.py`, `sampling.py`, `german_voice.py`) for a complete workflow:
  1. Sample or select words.
  2. Generate a German sentence and English translation.
  3. Synthesize speech for the German sentence.
  4. Serve results via JSON API.
  5. Track user interaction: words marked as "unknown" are given higher probability in future rounds.

---

## Workflow Overview

1. **Word Selection:** Either manually or using `sampling.py` for random frequency-based sampling.
2. **Sentence Generation:** `generate_sentence()` constructs a German sentence including the selected words and translates it to English.
3. **Speech Synthesis:** `synthesize_speech()` converts the German sentence into a `.wav` audio file.
4. **Web Interface:** Flask endpoints deliver sentences, translations, and audio to the front-end.
5. **Adaptive Learning:** Words that the user clicks as "unknown" are prioritized in future rounds for personalized vocabulary reinforcement.

---

## Key Techniques and Design Choices

- **AI-based Sentence Generation:** Uses GPT-4.1 to produce grammatically correct and contextually coherent sentences.
- **Probabilistic Word Sampling with Adaptivity:** Ensures realistic usage of words and prioritizes unknown words for personalized learning.
- **Neural TTS with Piper:** Provides high-quality German speech synthesis.
- **API-first Design:** Flask serves a RESTful API, enabling easy integration with web or mobile frontends.
- **Separation of Concerns:** Generation, sampling, and voicing are modular and reusable independently.

---

## Run the Code

python3 app.py

```
<img width="755" height="785" alt="Capture d’écran 2025-12-20 à 17 50 46" src="https://github.com/user-attachments/assets/52fe1824-7b3a-4240-89af-0cf5b27269c3" />




