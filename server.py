from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

# Store LED state
data = {"led": "off"}

@app.route('/')
def index():
    return render_template('page.html', led_state=data['led'])

@app.route('/led', methods=['POST'])
def control_led():
    state = request.form.get('state')
    if state not in ['On', 'Off']:
        return jsonify({"error": "Invalid state"}), 400
    data['led'] = state.lower()
    return jsonify({"led": data['led']})

@app.route('/led', methods=['GET'])
def get_led_state():
    return jsonify({"led": data['led']})

if __name__ == '__main__':
    PORT = 8000
    app.run(host='0.0.0.0', port=PORT)
