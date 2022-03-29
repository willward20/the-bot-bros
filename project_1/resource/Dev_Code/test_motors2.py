from gpiozero import PhaseEnableRobot
from gpiozero import LED
from time import sleep

robot = PhaseEnableRobot(left=(24,12), right=(25,13))
#motor1 = PhaseEnableMotor(24,12)
#motor_pin_1 = LED(22)
#motor_pin_1.on()
#motor2 = PhaseEnableMotor(25, 13)
#motor_pin_2 = LED(23)
#motor_pin_2.on()

#motor1.forward(0.5)
#motor2.backward(0.5)
pwm = 0.0
robot.forward(pwm)
try:
    while True:
        pwm = pwm+ 0.1
        robot.forward(pwm)
        print(pwm)
        sleep(5)
except KeyboardInterrupt:
    robot.stop()
