from gpiozero import PhaseEnableMotor
from gpiozero import LED
from time import sleep


motor1 = PhaseEnableMotor(24,12)
motor_pin_1 = LED(22)
motor_pin_1.on()

motor1.forward(0.7)

sleep (5)

motor1.stop()
