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
pi_puck_id = socket.gethostname().replace("pi-puck", "") if socket.gethostname().startswith("pi-puck") else '35'
max_range = 0.3
x = 0.0
y = 0.0
angle = 0.0
is_leader = False
target_x = 0.1
target_y = 0.1
ready = False

# Arena parameters
StartX     = 0.1
SweepEndX  = 1.9   # 2m arena minus 0.1 margin
ArenaMaxY  = 1.0

forward_speed=500
rotation_speed=300
wheel_step_to_cm = 0.01288  # 1 step ≈ 0.01288 cm
axle_radius_cm = 2.65       # 53 mm between wheels → r = 2.65 cm

# Row-sweeping globals
rowY       = None
spacing    = None
sweep_direction = 1  # 1=right, -1=left
puck_dict = {}
puck_pos_dict = {}

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
                # if robot_id == pi_puck_id:
                #     continue  
                msg_x = robot_data.get("x")
                msg_y = robot_data.get("y")

                dist = distance(x_self, y_self, msg_x, msg_y)
                if dist <= max_range:
                    puck_dict[robot_id] = robot_data

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
        is_leader = all(pi_puck_id <= rid for rid in puck_dict.keys())
        if is_leader:
            print(f"PiPuck {pi_puck_id} is the leader.")
        #     pipuck.epuck.set_leds_colour("green")
        # else:
        #     pipuck.epuck.set_leds_colour("red")
    except ValueError:
        print("No PiPucks available to determine leader.")
        is_leader = False
        # pipuck.epuck.set_leds_colour("red")
    

def extract_int(s):     return int(''.join(filter(str.isdigit, s)))
def normalize_angle_deg(a): return a % 360

def rotate_to_target():
    print(f'Rotating to target position: ({target_x}, {target_y})')
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

def drive_forward_stepwise(tx, ty, spd=forward_speed):
    global start_position
    x,y,_ = get_position()
    if start_position is None:
        start_position = (x,y)
    d = distance(x,y,tx,ty)
    print(f"[{pi_puck_id}] Driving→ ({x:.2f},{y:.2f})→({tx:.2f},{ty:.2f}) d={d:.3f}")
    if d < 0.1:
        pipuck.epuck.set_motor_speeds(0, 0)
        start_position = None
        return True
    pipuck.epuck.set_motor_speeds(spd, spd)
    return False
    
def rotate_to_angle(target_angle):
    angle_diff = (target_angle - angle + 540) % 360 - 180
    turn_speed = max(5 * abs(angle_diff), 100)
    if not (target_angle > angle + 2 or target_angle < angle - 2):
        # Move towards the target
        pipuck.epuck.set_motor_speeds(0, 0)
        #return STATE_START_WAIT
    if angle_diff > 0:
        pipuck.epuck.set_motor_speeds(turn_speed, -turn_speed)
    else:
        pipuck.epuck.set_motor_speeds(-turn_speed, turn_speed)
    return STATE_ROTATE_TO_90

def rotate_to_target_stepwise(x, y, ang, tx, ty, thresh=3.0):
    dx = tx - x
    dy = ty - y
    angle1 = math.degrees(math.atan2(dy, dx))
    targ_ang = (-angle1 + 90) % 360

    diff = (targ_ang - ang + 540) % 360 - 180
    print(f"[{pi_puck_id}] Rotating→ T:{targ_ang:.1f} C:{ang:.1f} Δ:{diff:.1f}")
    if abs(diff) < thresh:
        pipuck.epuck.set_motor_speeds(0, 0)
        return True
    spd = max(min(0.05*diff, rotation_speed), 100)
    if diff > 0:
        pipuck.epuck.set_motor_speeds(spd, -spd)
    else:
        pipuck.epuck.set_motor_speeds(-spd, spd)
    #pipuck.epuck.set_motor_speeds(spd if diff>0 else -spd,-spd if diff>0 else spd)
    return False
    
# States
STATE_START         = 0
STATE_WAIT_FOR_NEIGHBORS = 1
STATE_START_ROTATE  = 2
STATE_START_DRIVE   = 3
STATE_LINE_REACHED  = 4
STATE_START_SWEEP   = 5
STATE_SWEEP_DRIVE   = 6
STATE_ADVANCE_ROW   = 7
STATE_ROW_ROTATE    = 8
STATE_ROW_DRIVE     = 9
STATE_DONE          = 10
STATE_ROTATE_TO_90  = 11

current_state = STATE_START
role          = "UNKNOWN"
target_x      = None
target_y      = None
start_position = None

# Row-sweeping globals
rowY      = None
spacing   = None
sweep_direction = 1  # 1=right, -1=left
start_waiting = 50
try:
    for _ in range(1000):
        # TODO: Do your stuff here
        time.sleep(0.1)
        print(f'puck_dict: {puck_dict}')
        # print(f'target_x: {target_x}, target_y: {target_y}')
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
                    "ready": ready
                }
            })
        else:
            print("Position data not available.")
            
        all_ids = sorted(list(set(puck_dict.keys()) | {pi_puck_id}), key=extract_int)
                
        #if spacing is None and len(all_ids) >= 2:
        #    spacing = min(max_range*0.9, ArenaMaxY/len(all_ids))
        #    rowY    = 0.1
        #    total_sweeps = math.ceil(ArenaMaxY / spacing)
        #    sweeps_per_rbt = math.ceil(total_sweeps / len(all_ids))
        #    print(f"total_sweeps: {total_sweeps}, sweeps_per_rbt: {sweeps_per_rbt}, spacing: {spacing}")

        if current_state == STATE_START:
            print("Starting state...")
            current_state = STATE_WAIT_FOR_NEIGHBORS

        elif current_state == STATE_WAIT_FOR_NEIGHBORS:
            print(f"Waiting for neighbors... {len(puck_dict)} found.")
            if len(all_ids) < 2:
                current_state = STATE_START
                continue
            else:
                if spacing is None and len(all_ids) >= 1:
                    spacing = min(max_range*0.9, ArenaMaxY/len(all_ids))
                    rowY    = 0.1
                    total_sweeps = math.ceil(ArenaMaxY / spacing)
                    sweeps_per_rbt = math.ceil(total_sweeps / len(all_ids))
                    print(f"total_sweeps: {total_sweeps}, sweeps_per_rbt: {sweeps_per_rbt}, spacing: {spacing}")
                    
                role      = "LEADER" if int(pi_puck_id)==min(map(int,all_ids)) else "FOLLOWER"
                idx       = all_ids.index(pi_puck_id)
                target_x  = StartX
                target_y  = rowY + idx*spacing
                print(f"I am {role} idx={idx}, lineY={rowY:.2f}, target=({target_x:.2f},{target_y:.2f})")
                current_state = STATE_START_ROTATE
                        
        elif current_state == STATE_START_ROTATE:
            print(f"{pi_puck_id} STATE_START_ROTATE at Y={target_y:.2f}, direction={sweep_direction}")
            if rotate_to_target_stepwise(x,y,angle,target_x,target_y):
                current_state = STATE_START_DRIVE

        elif current_state == STATE_START_DRIVE:
            print(f"{pi_puck_id} STATE_START_DRIVE at Y={target_y:.2f}, direction={sweep_direction}")
            if drive_forward_stepwise(target_x,target_y):
                print(f"{pi_puck_id} formed line.")
                target_x = SweepEndX if sweep_direction == 1 else StartX         
                current_state = STATE_START_SWEEP

        elif current_state == STATE_START_SWEEP:
           print(f"{pi_puck_id} STATE_START_SWEEP at Y={target_y:.2f}, direction={sweep_direction}")
           if rotate_to_target_stepwise(x,y,angle,target_x,target_y):
              current_state = STATE_SWEEP_DRIVE

        elif current_state == STATE_SWEEP_DRIVE:
            print(f"{pi_puck_id} STATE_SWEEP_DRIVE at Y={target_y:.2f}, direction={sweep_direction}")
            if drive_forward_stepwise(target_x,target_y):
                print(f"{pi_puck_id} sweep row complete.")
                sweeps_per_rbt -= 1
                current_state = STATE_ADVANCE_ROW


        elif current_state == STATE_ADVANCE_ROW:
           print(f"{pi_puck_id} STATE_ADVANCE_ROW.")
           idx = all_ids.index(pi_puck_id)
           sweep_count = total_sweeps - sweeps_per_rbt
           next_row_index = idx + sweep_count * len(all_ids)
           next_row_y = next_row_index * spacing
           if next_row_y >= ArenaMaxY:
              current_state = STATE_DONE
           else:
              target_y = next_row_y
              current_state = STATE_ROW_ROTATE

        elif current_state == STATE_ROW_ROTATE:
            # Turn to new row position (Y changes, X remains)
            print(f"{pi_puck_id} STATE_ROW_ROTATE at Y={target_y:.2f}")
            if abs(target_y - rowY) < 0.05:
                current_state = STATE_ROW_DRIVE
            else:
                if rotate_to_target_stepwise(x, y, angle, x, target_y):
                    current_state = STATE_ROW_DRIVE

        elif current_state == STATE_ROW_DRIVE:
            print(f"{pi_puck_id} STATE_ROW_DRIVE at Y={target_y:.2f}, direction={sweep_direction}")
            if drive_forward_stepwise(x, target_y):
                sweep_direction *= -1
                target_x = SweepEndX if sweep_direction == 1 else StartX
                current_state = STATE_START_SWEEP

        elif current_state == STATE_DONE:
            pipuck.epuck.set_motor_speeds(0, 0)
            print(f"{pi_puck_id} DONE sweeping.")
            time.sleep(0.1)
            break
        
             
            

except KeyboardInterrupt:
    print("Interrupt detected!!")
finally:
    pipuck.epuck.set_motor_speeds(0,0)

# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  

