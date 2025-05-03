"""mutual_attraction_controller.py
Each robot broadcasts an empty message every step,
listens to neighbours, averages the direction vectors returned
by wb_receiver_get_emitter_direction(), and steers toward that
average (centroid) using differential drive.
"""

from controller import Robot
import numpy as np
import math
import random

# ---- robot & devices --------------------------------------------------------
robot       = Robot()
TIME_STEP   = int(robot.getBasicTimeStep())

MAX_WHEEL_VEL = 6.28        # half of e-puck max for stability

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

emitter   = robot.getDevice("emitter")
receiver  = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

# ---- random walk parameters -----------------------------------------------
DIRECTION_CHANGE_TIME = 2000  # 2 seconds
ROTATION_TIME = 500  # 0.5 seconds for rotation
last_direction_change = 0
current_direction = random.uniform(0, 2 * math.pi)
is_rotating = False
rotation_start_time = 0
rotate_left = True

# ---- helper ---------------------------------------------------------------
def diff_drive(theta, base_speed=MAX_WHEEL_VEL):
    """
    theta  : desired heading in robot local frame (rad).
             0  -> straight ahead
             +π -> straight back
             + left, - right
    returns (v_l, v_r)
    """
    # simple proportional controller
    K = 2.0                    # gain, tweak if it oscillates
    v_l = base_speed - K * theta
    v_r = base_speed + K * theta
    # clip to wheel limits
    v_l = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, v_l))
    v_r = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, v_r))
    return v_l, v_r

# ---- main loop ------------------------------------------------------------
while robot.step(TIME_STEP) != -1:
    current_time = robot.getTime() * 1000  # Convert to milliseconds

    # 1. broadcast heartbeat (content irrelevant)
    emitter.send(b"1")

    # 2. accumulate neighbour directions (local frame)
    dirs = []
    while receiver.getQueueLength() > 0:
        dirs.append(np.array(receiver.getEmitterDirection()))
        receiver.nextPacket()

    if dirs:
        # 3. centroid direction (mutual attraction rule)
        mean_vec = np.mean(dirs, axis=0)
        mean_vec[2] = 0.0                   # ignore z
        norm = np.linalg.norm(mean_vec)
        if norm > 1e-3:                     # safe guard div/0
            mean_vec /= norm
            theta = math.atan2(mean_vec[1], mean_vec[0])
            v_l, v_r = diff_drive(theta)
        else:
            v_l = v_r = 0.0                 # stacked? stop
    else:
        # 4. no neighbours: structured random walk
        if current_time - last_direction_change >= DIRECTION_CHANGE_TIME and not is_rotating:
            current_direction = random.uniform(0, 2 * math.pi)
            last_direction_change = current_time
            is_rotating = True
            rotation_start_time = current_time
            rotate_left = random.choice([True, False])
        
        if is_rotating:
            if current_time - rotation_start_time < ROTATION_TIME:
                if rotate_left:
                    v_l = -MAX_WHEEL_VEL * 0.5
                    v_r = MAX_WHEEL_VEL * 0.5
                else:
                    v_l = MAX_WHEEL_VEL * 0.5
                    v_r = -MAX_WHEEL_VEL * 0.5
            else:
                is_rotating = False
        else:
            v_l = v_r = MAX_WHEEL_VEL * 0.5

    # 5. apply wheel velocities
    left_motor.setVelocity(v_l)
    right_motor.setVelocity(v_r)
