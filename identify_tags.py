import paho.mqtt.client as mqtt
import struct
import time

print("==================================================")
print("             UWB Tag Identifier")
print("==================================================")
print("Power on your Rover. Then, turn on your mast tags ONE AT A TIME.")
print("This script will tell you which Tag ID (0, 1, or 2) is currently visible,")
print("along with its distance from the rover.")
print("==================================================")

import json
import os

config_path = os.path.join(os.path.dirname(__file__), "TerrainPlan", "anchor_config.json")
try:
    with open(config_path, 'r') as f:
        config = json.load(f)
        calibration = config.get("calibration", {}).get("per_anchor", {})
except Exception:
    calibration = {}

def parse_uwb_packet(data):
    anchors = {}
    if len(data) == 37 and data[0] == 0xAA and data[1] == 0x25 and data[2] == 0x01 and data[-1] == 0x55:
        checksum = sum(data[0:35]) & 0xFF
        if checksum == data[35]:
            for anchor_id in range(8):
                idx = 3 + (anchor_id * 4)
                dist_mm = struct.unpack('<I', data[idx:idx+4])[0]
                if 100 < dist_mm < 50000:
                    raw_dist = dist_mm / 1000.0
                    
                    # Apply calibration
                    aid_str = str(anchor_id)
                    scale = 1.0
                    offset = 0.0
                    if aid_str in calibration:
                        scale = calibration[aid_str].get("scale_factor", 1.0)
                        offset = calibration[aid_str].get("offset_mm", 0.0)
                    
                    corrected_dist = (raw_dist * scale) + (offset / 1000.0)
                    anchors[aid_str] = corrected_dist
    return anchors

def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("\n✓ Connected to MQTT! Listening for tags...")
        client.subscribe("rover/uwb/raw")

last_print = 0
def on_message(client, userdata, msg):
    global last_print
    anchors = parse_uwb_packet(msg.payload)
    if anchors and time.time() - last_print > 1.0:  # Print once per second
        print(f"\n--- Currently Visible Tags ---")
        for aid, dist in anchors.items():
            print(f"  Mast Tag ID {aid} : {dist:.2f} meters away")
        last_print = time.time()

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect("broker.hivemq.com", 1883, 60)

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nExiting...")
