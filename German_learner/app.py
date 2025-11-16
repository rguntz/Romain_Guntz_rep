from flask import Flask, render_template, jsonify, request, send_file
from german_learner.sentence_generator import generate_random_sentence
from german_learner.voicing.german_voice import synthesize_speech
import os

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/generate_german', methods=['POST'])
def generate_german():
    """Generate German sentence and audio"""
    try:
        data = request.json
        n_words = data.get('n_words', 5)
        words_to_review = data.get('words_to_review', [])
        print("words_to_review", words_to_review)
        
        # Generate both sentences
        german_sentence, english_sentence = generate_random_sentence(n_words, words_to_review)
        
        # Generate audio file
        audio_path = synthesize_speech(german_sentence)
        
        return jsonify({
            'success': True,
            'german_sentence': german_sentence,
            'english_sentence': english_sentence,
            'audio_ready': True  # Signal that audio is available
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/audio/german_sentence.wav')
def serve_audio():
    """Serve the generated audio file"""
    try:
        # Path should match OUTPUT_WAV_PATH from your config
        audio_path = 'data/voices/test.wav'  # Adjust if needed
        return send_file(audio_path, mimetype='audio/wav')
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 404

@app.route('/get_english', methods=['POST'])
def get_english():
    """Return the English translation"""
    try:
        data = request.json
        english_sentence = data.get('english_sentence', '')
        
        return jsonify({
            'success': True,
            'english_sentence': english_sentence
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

if __name__ == '__main__':
    # Create necessary directories
    os.makedirs('templates', exist_ok=True)
    os.makedirs('data/voices', exist_ok=True)
    app.run(debug=True, port=5000)