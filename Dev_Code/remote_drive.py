from gpiozero import PhaseEnableMotor
from gpiozero import LED
from time import sleep


motor1 = PhaseEnableMotor(24,12)
motor2 = PhaseEnableMotor(25, 13)


speed = 0.0
try:
    while speed < 0.9:
        speed = speed + 0.1
        motor1.backward(speed)
        motor2.backward(speed)
        sleep(1)
        print(speed)
    motor1.forward(0.99)
    motor2.forward(0.99)
    while True:
        print('nothing')

except KeyboardInterrupt:
    motor1.stop()
    motor2.stop()
