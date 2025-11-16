import openai
from german_learner.sampling.sampling import sample_words
from german_learner.generation.generation import generate_sentence
from .config import OPENAI_API_KEY, WORDS_JSON_FILE, TEMPERATURE
import random 

def generate_random_sentence(n_words=5, words_to_review=[]):
    """
    Sample words and generate a German sentence with English translation.
    
    Args:
        n_words (int): Number of words to sample for the sentence.
        words_to_review (list): Words to prioritize in the sentence.
    
    Returns:
        tuple: (german_sentence, english_sentence)
    """

    # Sample words from JSON
    words = sample_words(WORDS_JSON_FILE, n_words)
    
    # If there are words to review, combine them with sampled words
    if words_to_review:
        combined_words = list(set(words_to_review + words))  # remove duplicates
        # Resample n_words from the combined list
        if len(combined_words) > n_words:
            words = random.sample(combined_words, n_words)
        else:
            words = combined_words

    # Create client with API key from config
    client = openai.OpenAI(api_key=OPENAI_API_KEY)
    
    # Generate sentence with fixed temperature from config
    german_sentence, english_sentence = generate_sentence(client, words, TEMPERATURE)
    
    return german_sentence, english_sentence

