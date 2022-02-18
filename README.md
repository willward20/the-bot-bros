# the-bot-bros
UCA Robotics 2 Group -- Austin, Brandon, and Will

ssh ubuntu@192.168.0.145
Password: raspberry

2/18 -- Tested the motors today. The motors themselves have no problem spinning in either direction.
However, when we tried to use the backward() function on PhaseEnableMotor and PhaseEnableRobot,
the wheels did not respond well or at all depending on the speed. Cause is uncertain, but we know that
the motors are not the problem. It didn't matter which way the motors were wired into the board, forward()
function always worked and backward() never worked.
