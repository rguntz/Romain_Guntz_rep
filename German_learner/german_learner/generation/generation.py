def generate_sentence(client, words, temperature=0.7):
    """
    Generate a German sentence using the given words and its English translation.

    Args:
        client: OpenAI client instance.
        words (list of str): List of words to include in the sentence.
        temperature (float): Creativity/randomness of the model (default 0.7).

    Returns:
        tuple: (german_sentence, english_sentence)
    """
    if not words or len(words) == 0:
        raise ValueError("The words list must contain at least one word.")

    # Build the prompt dynamically
    word_list_str = ', '.join(f'"{w}"' for w in words)
    prompt = f"""
Create one natural and grammatically correct German sentence that uses exactly these words: 
{word_list_str}.

After that, provide the English translation of this sentence.

Format your response like this:

GERMAN: <German sentence>
ENGLISH: <English translation>

Respond only with this format, nothing else.
"""

    # Call the OpenAI API
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages=[
            {"role": "system", "content": "You are an assistant that generates German sentences and their English translations."},
            {"role": "user", "content": prompt}
        ],
        temperature=temperature
    )

    # Extract the output
    output = response.choices[0].message.content.strip()
    lines = output.splitlines()

    # Parse German and English sentences
    german_sentence = None
    english_sentence = None
    for line in lines:
        if line.startswith("GERMAN:"):
            german_sentence = line.replace("GERMAN: ", "").strip()
        elif line.startswith("ENGLISH:"):
            english_sentence = line.replace("ENGLISH: ", "").strip()

    if not german_sentence or not english_sentence:
        raise ValueError("Failed to parse the model output correctly.")

    return german_sentence, english_sentence

