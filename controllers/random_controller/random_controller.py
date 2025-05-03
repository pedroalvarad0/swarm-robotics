"""random_controller"""

from controller import Robot
import random
import math

robot = Robot()
name = robot.getName()
time_step = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Velocidad máxima del motor
MAX_SPEED = 6.28

# Tiempo entre cambios de dirección (en milisegundos)
DIRECTION_CHANGE_TIME = 2000  # 2 segundos
ROTATION_TIME = 500  # 0.5 segundos para rotar

# Variables para el control del movimiento
last_direction_change = 0
current_direction = random.uniform(0, 2 * math.pi)  # Dirección inicial aleatoria
is_rotating = False
rotation_start_time = 0
rotate_left = True  # Variable para controlar la dirección de rotación

while robot.step(time_step) != -1:
    current_time = robot.getTime() * 1000  # Convertir a milisegundos
    
    # Cambiar dirección si ha pasado el tiempo suficiente
    if current_time - last_direction_change >= DIRECTION_CHANGE_TIME and not is_rotating:
        current_direction = random.uniform(0, 2 * math.pi)
        last_direction_change = current_time
        is_rotating = True
        rotation_start_time = current_time
        rotate_left = random.choice([True, False])  # Elegir aleatoriamente la dirección de rotación
    
    # Si está rotando
    if is_rotating:
        if current_time - rotation_start_time < ROTATION_TIME:
            # Rotar en el lugar según la dirección elegida
            if rotate_left:
                left_motor.setVelocity(-MAX_SPEED * 0.5)
                right_motor.setVelocity(MAX_SPEED * 0.5)
            else:
                left_motor.setVelocity(MAX_SPEED * 0.5)
                right_motor.setVelocity(-MAX_SPEED * 0.5)
        else:
            is_rotating = False
    else:
        # Movimiento recto
        speed = MAX_SPEED * 0.5  # Usamos la mitad de la velocidad máxima para mejor control
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)