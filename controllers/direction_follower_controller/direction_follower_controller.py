"""direction_follower_controller"""

from controller import Robot
import math

robot = Robot()
name = robot.getName()
time_step = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Communication devices
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(time_step)

# Constants for motor control
MAX_SPEED = 6.28  # Maximum wheel speed

while robot.step(time_step) != -1:
    # emitter.send("message from " + name)

    direction = []
    left_speed = 0.0
    right_speed = 0.0

    while receiver.getQueueLength() > 0:
        direction = receiver.getEmitterDirection()
        print(f"{name}: detectando un vecino en direccion {direction}")
        receiver.nextPacket()

    if direction:  # If we received a direction
        # Normalize the direction vector
        magnitude = math.sqrt(direction[0]**2 + direction[1]**2)
        if magnitude > 0:
            normalized_direction = [direction[0]/magnitude, direction[1]/magnitude]
            
            # Convert direction to motor speeds
            # The x component affects forward/backward movement
            # The y component affects turning
            left_speed = MAX_SPEED * (normalized_direction[0] - normalized_direction[1])
            right_speed = MAX_SPEED * (normalized_direction[0] + normalized_direction[1])
            
            # Clamp speeds to valid range
            left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
            right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
    
    # Apply the calculated speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    

