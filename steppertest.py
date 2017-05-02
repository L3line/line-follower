#!/usr/bin/python3
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
 
import time
import atexit
import threading

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

def stepper_worker(stepper, numsteps, direction, style):
        stepper.step(numsteps, direction, style)



atexit.register(turnOffMotors)

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr = 0x60)

stepStyle = Adafruit_MotorHAT.INTERLEAVE
directionLeft = Adafruit_MotorHAT.FORWARD
directionRight = Adafruit_MotorHAT.FORWARD
stepRight = 200
stepLeft = 200


# 200 steps/rev, motor 1 = M1 and M2
rightMotor = mh.getStepper(200, 1)
# 2 = M3 and M4
leftMotor = mh.getStepper(200, 2)

motors = [leftMotor, rightMotor]

for motor in motors:
        motor.setSpeed(10)



st1 = threading.Thread(target=stepper_worker, args=(rightMotor, stepRight, directionRight, stepStyle))
st2 = threading.Thread(target=stepper_worker, args=(leftMotor, stepLeft, directionLeft, stepStyle))
st1.start()
st2.start()

    
