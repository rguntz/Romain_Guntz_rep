from german_learner.sentence_generator import generate_random_sentence

def main():
    # Example: generate a sentence with 5 words
    n_words = 5
    german, english = generate_random_sentence(n_words)
    
    print("GERMAN:", german)
    print("ENGLISH:", english)

if __name__ == "__main__":
    main()
