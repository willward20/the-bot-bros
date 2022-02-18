# the-bot-bros
UCA Robotics 2 Group -- Austin, Brandon, and Will

ssh ubuntu@192.168.0.145
Password: raspberry

2/18 -- Tested the motors today. The motors themselves have no problem spinning in either direction.
However, when we tried to use the backward() function on PhaseEnableMotor and PhaseEnableRobot,
the wheels did not respond well or at all depending on the speed. Cause is uncertain, but we know that
the motors are not the problem. It didn't matter which way the motors were wired into the board, forward()
function always worked and backward() never worked.

We need a longer ribbon ribbon cable. Amazon has some options for 2 feet to 2 meter long cables:
https://www.amazon.com/Adafruit-Flex-Cable-Raspberry-Camera/dp/B00M4DAQH8
https://www.amazon.com/dp/B08WLQGKVT/ref=sspa_dk_detail_3?psc=1&pd_rd_i=B08WLQGKVT&pd_rd_w=Ts0Cj&pf_rd_p=b9951ce4-3bd8-4b04-9123-0fda35d6155e&pd_rd_wg=8Bl4x&pf_rd_r=CW9WWQRK7J2SY7JB2A3V&pd_rd_r=496e3496-3d82-4331-87d9-9898ef402ac8&s=pc&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyUVJWWDcwSEpNMjJHJmVuY3J5cHRlZElkPUEwOTEwMDk0MkZFUEM3TUo1R1pNOSZlbmNyeXB0ZWRBZElkPUEwNTMzNzY1M09XWVBXQUk0TVdZViZ3aWRnZXROYW1lPXNwX2RldGFpbCZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=
