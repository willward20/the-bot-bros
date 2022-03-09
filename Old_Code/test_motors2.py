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
robot.forward(0.7)
sleep(20)
#robot.stop()
robot.backward(.7)
sleep (20)

#motor1.stop()
#motor2.stop()
robot.stop()
