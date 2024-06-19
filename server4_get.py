import requests
import json
import os
from datetime import datetime

api_url_get_positions = 'http://89.58.6.184:8000/fre2024/task4/get-positions'
api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'

def get_positions():
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:
        response = requests.get(api_url_get_positions, headers=headers)
        response.raise_for_status()
        positions = response.json()
        print("Received positions:", positions)

        # Save to JSON file
        save_to_json(positions)

    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")

def save_to_json(data):
    # Path to save the JSON file in the Downloads folder
    download_folder = os.path.join(os.path.expanduser('~'), 'Downloads')
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_file_path = os.path.join(download_folder, f"weed_positions_{timestamp}.json")

    with open(json_file_path, 'w') as json_file:
        json.dump(data, json_file, indent=4)
    print(f"Data saved to {json_file_path}")

if __name__ == '__main__':
    get_positions()
