import wave
from piper import PiperVoice, SynthesisConfig
from ..config import PIPER_MODEL_PATH, OUTPUT_WAV_PATH, VOLUME, LENGTH_SCALE, NOISE_SCALE, NOISE_W_SCALE, NORMALIZE_AUDIO
    

def synthesize_speech(text: str) -> str:
    """
    Synthesizes speech from text using a Piper voice model and saves it to a WAV file.
    
    Args:
        text (str): Text to synthesize.
    
    Returns:
        str: Path to the saved WAV file.
    """
    # Load the voice
    voice = PiperVoice.load(PIPER_MODEL_PATH)

    # Configure synthesis using parameters from config
    syn_config = SynthesisConfig(
        volume=VOLUME,
        length_scale=LENGTH_SCALE,
        noise_scale=NOISE_SCALE,
        noise_w_scale=NOISE_W_SCALE,
        normalize_audio=NORMALIZE_AUDIO
    )

    # Synthesize and save WAV
    with wave.open(OUTPUT_WAV_PATH, "wb") as wav_file:
        voice.synthesize_wav(text, wav_file, syn_config=syn_config)

    return OUTPUT_WAV_PATH


if __name__ == "__main__":
    # Example text to synthesize
    example_text = "Guten Morgen, wie geht es dir?"

    # Call the synthesis function
    output_file = synthesize_speech(example_text)

    print(f"WAV file successfully saved at: {output_file}")
