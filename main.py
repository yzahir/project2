import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import random

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = '14'
max_range = 0.5
x = 0.0
y = 0.0

puck_pos_dict = {}
puck_dict = {}

def distance(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")
    client.subscribe("robots/all")

# function to handle incoming messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if msg.topic == "robot_pos/all":
            # Check if the message is a dictionary
            puck_pos_dict.update(data)
        if msg.topic == "robots/all":
            msg_x = data.get(pi_puck_id, {}).get("x")
            
    except json.JSONDecodeError:

        print(f'invalid json: {msg.payload}')

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)

# Set the robot's speed, e.g. with
pipuck.epuck.set_motor_speeds(1000,-1000)

def get_position(id=pi_puck_id):
    data = puck_pos_dict.get(id)
    if data:
        pos = data.get('position')
        if pos:
            x = pos[0]
            y = pos[1]
            angle = data.get('angle')
            return x, y, angle
    else:
        print(f"No data for PiPuck ID: {id}")
    return None, None, None

def publish_data(packet):
    client.publish("robots/all", json.dumps(packet))

try:
    for _ in range(1000):
        # TODO: Do your stuff here
        x, y, angle = get_position()
        if x is not None and y is not None:
            publish_data({
                pi_puck_id: {
                    "x": x,
                    "y": y,
                    "angle": angle,
                    "sensors": {
                        "temperature": random.randint(0,50),
                        "humidity": random.randint(0,100),
                        "light": random.randint(0,100)
                    }
                }
            })
        else:
            print("Position data not available.")
        time.sleep(1)
        
except KeyboardInterrupt:
    print("Interrupt detected!!")
finally:
    pipuck.epuck.set_motor_speeds(0,0)

# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
