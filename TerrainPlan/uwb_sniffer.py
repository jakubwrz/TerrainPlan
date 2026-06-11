import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print("✓ Connected to MQTT broker!")
    print("Listening for raw UWB data on 'rover/uwb/raw'...\n")
    client.subscribe("rover/uwb/raw")

def on_message(client, userdata, msg):
    raw_data = msg.payload
    
    # Print the raw hex representation
    hex_str = " ".join([f"{b:02X}" for b in raw_data])
    
    # Print the ASCII representation (replacing non-printable characters with '.')
    ascii_str = "".join([chr(b) if 32 <= b <= 126 else "." for b in raw_data])
    
    print(f"[{len(raw_data)} bytes]")
    print(f"HEX: {hex_str}")
    print(f"TXT: {ascii_str}\n")

print("Initializing UWB Sniffer...")
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("broker.hivemq.com", 1883, 60)
client.loop_forever()
