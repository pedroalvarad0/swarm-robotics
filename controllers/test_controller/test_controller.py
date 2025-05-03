"""Manual controller for e-puck robot"""

from controller import Robot, Keyboard

robot = Robot()
name = robot.getName()
time_step = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Establecer una velocidad constante (en radianes por segundo)
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Keyboard setup
keyboard = Keyboard()
keyboard.enable(time_step)

# Velocidad base del robot (rad/s)
MAX_SPEED = 6.28  # Velocidad máxima del e-puck

# Communication devices
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(time_step)

print("Control manual del e-puck:")
print("Flechas: Mover el robot")
print("Espacio: Detener")
print("Q: Salir")

while robot.step(time_step) != -1:
    # Leer tecla presionada
    key = keyboard.getKey()
    
    # Inicializar velocidades
    left_speed = 0
    right_speed = 0
    
    # Control de movimiento
    if key == Keyboard.UP:
        # Avanzar
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
    elif key == Keyboard.DOWN:
        # Retroceder
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED
    elif key == Keyboard.LEFT:
        # Girar a la izquierda
        left_speed = -MAX_SPEED/2
        right_speed = MAX_SPEED/2
    elif key == Keyboard.RIGHT:
        # Girar a la derecha
        left_speed = MAX_SPEED/2
        right_speed = -MAX_SPEED/2
    elif key == ord('Q'):
        # Salir
        break
    
    # Aplicar velocidades a los motores
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # Mantener la comunicación
    emitter.send("message from " + name)

    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        direction = receiver.getEmitterDirection()
        print(f"{name}: detectando un vecino en direccion {direction}")
        receiver.nextPacket()