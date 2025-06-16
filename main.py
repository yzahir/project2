import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import random
import math
import socket

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = socket.gethostname().replace("pi-puck", "") if socket.gethostname().startswith("pi-puck") else '17'
max_range = 0.3
x = 0.0
y = 0.0
angle = 0.0
is_leader = False
target_x = 0.1
target_y = 0.5
forward_speed=500
rotation_speed=300
wheel_step_to_cm = 0.01288  # 1 step ≈ 0.01288 cm
axle_radius_cm = 2.65       # 53 mm between wheels → r = 2.65 cm

puck_pos_dict = {}
puck_dict = {}

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def remove_out_of_range_robots(puck_dict, robots_in_range):
    to_remove = [rid for rid in puck_dict if rid not in robots_in_range]
    for robot_id in to_remove:
        del puck_dict[robot_id]
        print(f'Removed Robot {robot_id} is no longer in range')

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
            puck_pos_dict.update(data)

        if msg.topic == "robots/all":
            x_self, y_self, _ = get_position()
            for robot_id, robot_data in data.items():
                if robot_id == pi_puck_id:
                    continue  
                msg_x = robot_data.get("x")
                msg_y = robot_data.get("y")

                dist = distance(x_self, y_self, msg_x, msg_y)
                if dist < max_range:
                    puck_dict[robot_id] = robot_data

    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')


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
#pipuck.epuck.set_motor_speeds(1000,-1000)

def get_position(id=pi_puck_id):
    global x, y

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

def set_leader():
    global is_leader
    try:
        is_leader = int(pi_puck_id) <= min(puck_dict.keys(), key=int)
        if is_leader:
            print(f"PiPuck {pi_puck_id} is the leader.")
        #     # pipuck.epuck.set_leds_colour("green")
        # else:
        #     # pipuck.epuck.set_leds_colour("red")
    except ValueError:
        print("No PiPucks available to determine leader.")
        is_leader = False
        # pipuck.epuck.set_leds_colour("red")
    

def move_to(target_x, target_y):
    current_x, current_y, angle = get_position()
    print(f"Current position: ({current_x}, {current_y}), angle: {angle}")
    if current_x is None or current_y is None or angle is None:
        print("Current position or angle not available.")
        return

    dx = target_x - current_x
    dy = target_y - current_y
    distance_to_target = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.degrees(math.atan2(dy, dx)) 

    angle_diff = (angle_to_target - angle + 180) % 360 - 180

    # Calculate angular speed in deg/sec
    wheel_speed_cm_s = rotation_speed * wheel_step_to_cm
    angular_speed_deg_s = (wheel_speed_cm_s / axle_radius_cm) * (180 / math.pi)


    # Rotate
    rotation_time = abs(angle_diff) / angular_speed_deg_s
    if angle_diff > 0:
        pipuck.epuck.set_motor_speeds(-rotation_speed, rotation_speed)  
    else:
        pipuck.epuck.set_motor_speeds(rotation_speed, -rotation_speed)  
    time.sleep(rotation_time)
    pipuck.epuck.set_motor_speeds(0, 0)  

    # Move forward
    linear_speed_cm_s = forward_speed * wheel_step_to_cm
    move_time = (distance_to_target * 100) / linear_speed_cm_s  # m → cm
    pipuck.epuck.set_motor_speeds(forward_speed, forward_speed)
    time.sleep(move_time)
    pipuck.epuck.set_motor_speeds(0, 0)  # Stop

    print(f"Moved to target position: ({target_x}, {target_y}) from ({current_x}, {current_y})")


def rotate_to_target():
    angle1 = math.degrees(math.atan2(target_y - y, target_x - x))
    # now get the angle from the y-axis to the target
    target_angle = (-angle1 + 90) % 360
    print(f"Target Angle: {target_angle}")
    if not (target_angle > angle + 5 or target_angle < angle - 5):
        # Move towards the target
        pipuck.epuck.set_motor_speeds(forward_speed, forward_speed)
        return STATE_START_DRIVE
    else:
        # Turn towards the target
        # Calculate the smallest difference between angles (handling wrap-around)
        angle_diff = (target_angle - angle + 540) % 360 - 180
        turn_speed = max(5 * abs(angle_diff), 100)
        if angle_diff > 0:
            pipuck.epuck.set_motor_speeds(turn_speed, -turn_speed)
        else:
            pipuck.epuck.set_motor_speeds(-turn_speed, turn_speed)
        return STATE_START_ROTATE
    
STATE_START = 0
STATE_START_ROTATE = 1
STATE_START_DRIVE = 2
current_state = STATE_START
start_waiting = 50
try:
    for _ in range(1000):
        # TODO: Do your stuff here
        print(f'puck_dict: {puck_dict}')
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
                    },
                    "target_found": False,
                }
            })
        else:
            print("Position data not available.")
        set_leader()
        if current_state == STATE_START:
            print("Starting state...")
            if start_waiting > 0:
                start_waiting -= 1
            else:
                if is_leader:
                    current_state = STATE_START_ROTATE
        elif current_state == STATE_START_ROTATE:
            # Rotate to face the target
            current_state = rotate_to_target()
        elif current_state == STATE_START_DRIVE:
            # Check if we've reached the target
            if abs(target_x - x) < 0.1 and abs(target_y - y) < 0.1:
                # Stop moving towards the target
                current_state = STATE_START
                pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(0.1)
        
             
            

except KeyboardInterrupt:
    print("Interrupt detected!!")
finally:
    pipuck.epuck.set_motor_speeds(0,0)

# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
