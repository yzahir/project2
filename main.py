import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import random
import math

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = '40'
max_range = 0.3
x = 0.0
y = 0.0
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
            robots_in_range = set()
            for robot_id, robot_data in data.items():
                if robot_id == pi_puck_id:
                    continue  
                msg_x = robot_data.get("x")
                msg_y = robot_data.get("y")

                dist = distance(x_self, y_self, msg_x, msg_y)
                if dist < max_range:
                    puck_dict[robot_id] = robot_data
                    robots_in_range.add(robot_id)
                
            remove_out_of_range_robots(puck_dict, robots_in_range)

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
    is_leader = pi_puck_id == min(puck_pos_dict.keys())
    if is_leader:
        print(f"PiPuck {pi_puck_id} is the leader.")
        pipuck.epuck.set_leds_colour("green")
    return is_leader

def move_to(target_x, target_y):
    # 1) Read pose
    x0, y0, angle0 = get_position()
    if x0 is None:
        return

    # 2) Compute desired heading & distance
    dx = target_x - x0
    dy = target_y - y0
    dist  = math.hypot(dx, dy)
    head  = math.degrees(math.atan2(dy, dx))
    diff  = (head - angle0 + 180) % 360 - 180

    # 3) Compute rotation time
    v_r = rotation_speed * wheel_step_to_cm
    v_l = -v_r
    track_width_cm = axle_radius_cm              # full width between wheels
    omega_deg_s = ((v_r - v_l) / track_width_cm) * (180 / math.pi)
    t_rot = abs(diff) / omega_deg_s

    # 4) Rotate wheels in opposite directions
    if diff > 0:
        pipuck.epuck.set_motor_speeds(-rotation_speed, rotation_speed)
    else:
        pipuck.epuck.set_motor_speeds( rotation_speed, -rotation_speed)
    time.sleep(t_rot)
    pipuck.epuck.set_motor_speeds(0, 0)

    # 5) Drive forward at forward_speed
    lin_cm_s = forward_speed * wheel_step_to_cm
    t_forw   = (dist * 100) / lin_cm_s
    pipuck.epuck.set_motor_speeds(forward_speed, forward_speed)
    time.sleep(t_forw)
    pipuck.epuck.set_motor_speeds(0, 0)

    print(f"→ rotated {diff:.1f}°, then drove {dist:.2f} m")


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
                    }
                }
            })
        else:
            print("Position data not available.")
        time.sleep(1)
        
        #if set_leader():
        move_to(0.3, 0.5)
            

except KeyboardInterrupt:
    print("Interrupt detected!!")
finally:
    pipuck.epuck.set_motor_speeds(0,0)

# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
