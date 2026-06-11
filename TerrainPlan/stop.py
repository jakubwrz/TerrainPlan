import paho.mqtt.client as mqtt
import time

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker. Sending STOP command...")
    client.publish("rover/motors/commands", "0,0")
    print("STOP command sent (0,0).")
    time.sleep(1)
    client.disconnect()

client = mqtt.Client()
client.on_connect = on_connect
print("Connecting to broker.hivemq.com...")
client.connect("broker.hivemq.com", 1883, 60)
client.loop_forever()
