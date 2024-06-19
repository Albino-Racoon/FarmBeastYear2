from flask import Flask, request, jsonify
import threading
import requests
import time

app = Flask(__name__)

# URL REST API-ja za pridobivanje lokacij plevela (task 4)
api_url_get_positions = 'http://89.58.6.184:8000/fre2024/task4/get-positions'
# API ključ za avtentifikacijo
api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'

# Globalni seznam za shranjevanje prejetih podatkov
received_positions = []


def fetch_positions_continuously():
    """ Neprekinjeno pridobiva podatke vsakih nekaj sekund. """
    global received_positions
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    while True:
        try:
            response = requests.get(api_url_get_positions, headers=headers)
            if response.status_code == 200:
                data = response.json()
                received_positions.extend(data)
                print("Received positions:", data)
            else:
                print(f"Failed to get positions: {response.status_code}, {response.text}")
        except requests.exceptions.RequestException as e:
            print("Request failed:", e)

        time.sleep(5)  # Počakaj 5 sekund pred naslednjo zahtevo


@app.route('/receive', methods=['POST'])
def receive_data():
    global received_positions
    data = request.json
    received_positions.append(data)
    print("Received data:", data)
    return jsonify({"status": "success", "message": "Data received successfully"}), 200


@app.route('/data', methods=['GET'])
def get_data():
    global received_positions
    return jsonify(received_positions), 200


if __name__ == '__main__':
    # Zaženi neprekinjeno pridobivanje podatkov v ozadju
    threading.Thread(target=fetch_positions_continuously, daemon=True).start()
    # Zaženi Flask strežnik
    app.run(debug=True, port=5000)
