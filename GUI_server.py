import tkinter as tk
from flask import Flask, request, jsonify
import json
import threading

app = Flask(__name__)

# Seznam za shranjevanje podatkov
data_list = []

# Nastavitev Tkinter okna
root = tk.Tk()
root.title("Data Visualization")

# Uporaba tkinter Variable za shranjevanje podatkov
data_var = tk.StringVar()
data_var.set("No data received yet.")

# Label za prikaz podatkov
label = tk.Label(root, textvariable=data_var, font=('Helvetica', 18), height=2)
label.pack(pady=20, padx=20)

@app.route('/receive', methods=['POST'])
def receive_data():
    data = request.json
    if data and 'data' in data:
        try:
            parsed_data = json.loads(data['data'])
            if 'topic' in parsed_data and parsed_data['topic'] == "/farmbeast/odom":
                msg = parsed_data['message']
                x_line = next(line for line in msg.split('\n') if 'x:' in line)
                linear_x = float(x_line.split('x:')[1].strip())
                time = float(parsed_data['time'])

                # Dodajanje podatkov v seznam in osvežitev GUI
                new_data = f"Time: {time}, X: {linear_x}"
                data_list.append(new_data)
                root.after(0, data_var.set, new_data)  # Varno posodabljanje GUI iz glavne niti

                return jsonify({"status": "success", "message": "Data received"}), 200
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 400

    return jsonify({"status": "error", "message": "Bad request"}), 400

def run_flask():
    app.run(debug=True, port=5000, use_reloader=False)

if __name__ == '__main__':
    # Zagon Flask aplikacije v ločeni niti
    threading.Thread(target=run_flask, daemon=True).start()
    # Zagon Tkinter glavne zanke v glavni niti
    root.mainloop()
