import json
import random

def sample_words(json_file, n):
    """
    Sample n words from a German word frequency JSON file according to their probability.

    Args:
        json_file (str): Path to the JSON file with 'word' and 'probability' fields.
        n (int): Number of words to sample.

    Returns:
        list: A list of sampled words.
    """
    # Load JSON data
    with open(json_file, "r", encoding="utf-8") as f:
        words_list = json.load(f)

    # Extract words and probabilities
    words = [entry["word"] for entry in words_list]
    probabilities = [entry["probability"] for entry in words_list]

    # Sample n words according to probabilities (with replacement)
    sampled_words = random.choices(words, weights=probabilities, k=n)

    return sampled_words