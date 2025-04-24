"""circle_controller – beacon + flood‑fill pattern formation
Each e‑puck runs the *same* controller.

• The robot named "beacon" stays at the centre (level 0) and only broadcasts.
• All other robots compute their hop‑count level = 1 + min(levels received).
• They move until their level equals LEVEL_TARGET (≈ desired circle radius / IR range).

Assumptions
-----------
– World size : 1 m × 1 m flat arena.
– We set the desired circle radius with CIRCLE_RADIUS (you can tune it).
– Emitter/Receiver devices are of type "infra‑red", range ≈ 0.25 m, aperture 6.28 rad.
– The standard e‑puck wheel motors are named "left wheel motor" / "right wheel motor".
"""

from controller import Robot
import math

# ---------- hyper‑parameters ---------- #
TIME_STEP          = 64                # ms – Webots default for e‑puck
MAX_WHEEL_SPEED    = 3.14              # rad/s (half of 6.28 for stability)
CIRCLE_RADIUS      = 0.35              # metres – <‑‑ change at will
IR_RANGE           = 0.25              # metres – must match Emitter.range
LEVEL_TARGET       = round(CIRCLE_RADIUS / IR_RANGE)
STUCK_SPIN_SPEED   = 0.5               # rad/s – slow search when no packets
PROX_THRESHOLD     = 80.0              # proximity raw value to start repulsion

# ---------- robot & devices ---------- #
robot = Robot()
name  = robot.getName()
is_beacon = (name.lower() == "beacon")

# Motors
left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))        # velocity‑controlled
    m.setVelocity(0.0)

# Communication devices
emitter   = robot.getDevice("emitter")
receiver  = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

# Proximity sensors (for repulsion / obstacle avoidance)
ps = [robot.getDevice(f"ps{i}") for i in range(8)]
for s in ps:
    s.enable(TIME_STEP)

# ---------- state ---------- #
level = 0 if is_beacon else 255        # 255 ≈ ∞
bearing_to_min = 0.0                   # radian: direction of minimum level neighbour

def drive(drive_speed, turn_speed):
    """Set differential wheel speeds.
    drive_speed (m/s equiv) projected to wheel rad/s with wheel_radius ~ 0.02 m,
    but here we scale directly with MAX_WHEEL_SPEED for simplicity."""
    left  = drive_speed - turn_speed
    right = drive_speed + turn_speed
    left_motor.setVelocity(max(min(left,  MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED))
    right_motor.setVelocity(max(min(right, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED))

# ---------- main loop ---------- #
while robot.step(TIME_STEP) != -1:
    # --- 1. receive packets & find neighbour with lowest level --- #
    min_level = 255
    bearing_to_min = None

    while receiver.getQueueLength() > 0:
        # Receiver.getString() decodes the payload as UTF‑8 text.
        # We previously sent a single byte; now we send the level as a UTF‑8 string.
        try:
            data_str   = receiver.getString()          # e.g. "5"
            neigh_level = int(data_str)
        except ValueError:
            # malformed packet – skip
            receiver.nextPacket()
            continue

        direction = receiver.getEmitterDirection()     # 3‑vector (x,y,z)
        receiver.nextPacket()

        if neigh_level < min_level:
            min_level = neigh_level
            # Convert (x,z) to signed bearing [‑π, π]
            bearing_to_min = -math.atan2(direction[0], direction[2])

    # --- 2. update own level (flood‑fill) --- #
    if not is_beacon and min_level < 255:
        # 1 + min(level neighbours)
        level = min(level, 1 + min_level)

    # --- 3. broadcast current level --- #
    emitter.send(str(level).encode('utf‑8'))

    # --- 4. motion control --- #
    if is_beacon:
        drive(0.0, 0.0)                  # stay put
        continue

    level_error = level - LEVEL_TARGET

    # 4a. If no packets were heard, spin slowly to search for neighbours
    if bearing_to_min is None:
        drive(0.0, STUCK_SPIN_SPEED)
        continue

    # 4b. Determine desired direction (sign of level_error)
    # inward  -> follow bearing to min‑level neighbour
    # outward -> opposite direction
    desired_bearing = bearing_to_min if level_error > 0 else (bearing_to_min + 3.14159)

    # Simple P‑controller on heading
    HEADING_KP = 2.0
    turn = HEADING_KP * desired_bearing
    drive_speed = 0.5                    # forward base speed (rad/s wheel equiv)
    drive(drive_speed, turn)

    # 4c. Repulsion for spacing when already on the ring
    if level_error == 0:
        # If front proximity too high, back off a little
        front_left  = ps[0].getValue()
        front_right = ps[7].getValue()
        if max(front_left, front_right) > PROX_THRESHOLD:
            drive(-0.5, 0.0)             # short reverse
