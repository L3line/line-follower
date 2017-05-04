#!/usr/bin/python3
from Adafruit_MotorHAT import Adafruit_MotorHAT, \
                              Adafruit_DCMotor, \
                              Adafruit_StepperMotor
import time
import atexit
import threading
import numpy as np

mh = Adafruit_MotorHAT(addr=0x60)
INTERLEAVE = Adafruit_MotorHAT.INTERLEAVE
SINGLE = Adafruit_MotorHAT.SINGLE
DOUBLE = Adafruit_MotorHAT.DOUBLE
MICROSTEP = Adafruit_MotorHAT.MICROSTEP

def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)


class stepperDrive(object):
    ''' Class to handle motor control

    '''

    def __init__(self, stepsPerRev=200, style=INTERLEAVE,
                 wheelRad=0.04, wheelBase=0.144):
        self.rightMotor = mh.getStepper(stepsPerRev, 1)
        self.leftMotor = mh.getStepper(stepsPerRev, 2)
        multipliers = {SINGLE: 1,
                       DOUBLE: 1,
                       MICROSTEP: 8,
                       INTERLEAVE: 2}
        self.multiplier = multipliers[style]
        self.steps = stepsPerRev
        self.wheelRad = wheelRad
        self.wheelBase = wheelBase
        self.style = style
        self.interrupt = threading.Event()
        self.interrupt.clear()

    def stepper_worker(self, motor, vel, duration):
        ''' Worker method to drive stepper motor

        '''
        if vel > 0:
            direction = Adafruit_MotorHAT.FORWARD
        elif vel == 0:
            return
        else:
            direction = Adafruit_MotorHAT.BACKWARD
        # Determine needed wheel rotation speed
        w = abs(vel/self.wheelRad)
        # determine steps per radian
        stepRad = self.multiplier*self.steps/(2*np.pi)

        stepwait = 1/(stepRad*w)
        numSteps = int(duration/stepwait)
        print(numSteps, stepwait, numSteps*stepwait)
        if numSteps == 0:
            return
        for i in range(int(duration/stepwait)):
            if not self.interrupt.is_set():
                lateststep = motor.oneStep(direction, self.style)
                time.sleep(stepwait)
            else:
                break

        if (self.style == MICROSTEP):
            # this is an edge case, if between full steps,just keep going
            # so we end on a full step
            while (lateststep != 0) and (lateststep != MICROSTEP):
                lateststep = motor.oneStep(direction, self.style)
                time.sleep(stepwait)

    def driveBlocking(self, commands):
        ''' blocking method to drive motosrs

        '''
        for driveCmd in commands:
            if not self.interrupt.is_set():
                self.lDrv = threading.Thread(target=self.stepper_worker,
                                             args=(self.leftMotor,
                                                   driveCmd[0],
                                                   driveCmd[2]))
                self.rDrv = threading.Thread(target=self.stepper_worker,
                                             args=(self.rightMotor,
                                                   driveCmd[1],
                                                   driveCmd[2]))
                self.lDrv.start()
                self.rDrv.start()
                self.lDrv.join()
                self.rDrv.join()

    def drive(self, commands):
        ''' Wrapper for driveBlocking that doesn't block, kills current command set when called

        '''
        try:
            while self.driveThread.is_alive():
                self.interrupt.set()
            self.interrupt.clear()
        except:
            pass
        self.driveThread = threading.Thread(target=self.driveBlocking,
                                            args=([commands]))
        self.driveThread.start()


# recommended for auto-disabling motors on shutdown!
atexit.register(turnOffMotors)

# Testing code, only runs if steppers.py is executed directly
if __name__ == "__main__":
    drive = stepperDrive()
    drive.drive([[0.01, -0.01, 5]])
    time.sleep(2)
    drive.drive([[0.01, 0.01, 5]])
    time.sleep(10)
