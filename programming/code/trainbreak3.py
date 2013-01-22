# ; -*- mode: Python; eval: (setq python-command "python2"); -*-

import sys
import time
from pycsp.parallel import *

acc_threshold = 20
epsilon = 0.1

class NewSpeedException(Exception):
    def __init__(self, obs_speed):
        self.obs_speed = obs_speed

    def getObsSpeed(self):
        return self.obs_speed

@process
def motor(to_control, from_control, skew=1):
    wheel_speed = 0

    while True:
        target_speed = from_control()
        if abs(target_speed - wheel_speed) > acc_threshold:
            to_control(target_speed)
        elif target_speed < wheel_speed - epsilon:
            wheel_speed -= skew
            to_control(wheel_speed)
        elif target_speed > wheel_speed + epsilon:
            wheel_speed += skew
            to_control(wheel_speed)
        else:
            wheel_speed = target_speed
            to_control(target_speed)
        

@process
def control(to_central, from_central, to_motor, from_motor, to_other, from_other):
    observed_speed = 0

    while True:
        try:
            target_speed = from_central()
            
            observed_speed = go_to_speed(from_central, to_motor, from_motor, to_other, from_other, observed_speed, target_speed)
            to_central("Successfully reached speed " + str(observed_speed))
        except NewSpeedException as e:
            observed_speed = e.getObsSpeed()

def go_to_speed(from_central, to_motor, from_motor, to_other, from_other, observed_speed, target_speed):
    #print "new go_to_speed. os: " + str(observed_speed) + ", ts: " + str(target_speed)
    while observed_speed != target_speed:
        g, msg = AltSelect(InputGuard(from_central),
                           OutputGuard(to_motor, target_speed))

        if g == from_central:
            print "New target!"
            raise NewSpeedException(observed_speed)

        new_obs_speed = from_motor()
        print "new_obs_speed: " + str(new_obs_speed)
        
        if abs(new_obs_speed - observed_speed) > 1:
            observed_speed = go_to_speed(from_central, to_motor, from_motor, to_other, from_other, observed_speed, (target_speed - observed_speed) / 2 + observed_speed)
        else:
            observed_speed = new_obs_speed
            
        other_speed = exchange_speeds(observed_speed, to_other, from_other)
        if other_speed != observed_speed: 
            print "Out of sync! obs: " + str(observed_speed) + ", os: " + str(other_speed)
            if observed_speed < other_speed <= target_speed or target_speed <= other_speed < observed_speed:
                print "We're behind, going ahead!"
            else:
                print "We're ahead! Waiting!"
                while True:
                    g, msg = AltSelect(InputGuard(from_central),
                                       OutputGuard(to_motor, observed_speed))
                    if g == from_central:
                        print "New target (while waiting)"
                        raise NewSpeedException(observed_speed)
                    # We know we're not spinningu
                    observed_speed = from_motor()
                    if exchange_speeds(observed_speed, to_other, from_other) >= observed_speed:
                        break

    return observed_speed

def aequal(n, m):
    print epsilon
    print n
    print m
    return abs(n-m) < epsilon

def exchange_speeds(observed_speed, to_other, from_other):
    ch_end, msg = AltSelect(InputGuard(from_other),
                            OutputGuard(to_other, observed_speed))
    if ch_end == from_other:
        to_other(observed_speed)
        return msg
    else:
        return from_other()

@process
def central(to_control1, from_control1, to_control2, from_control2):
    to_control1(100)
    to_control2(100)
    time.sleep(0.01)
    print from_control1()
    print from_control2()
    retire(to_control1)
    retire(from_control1)
    retire(to_control2)
    retire(from_control2)

def main():
    central_to_control1 = Channel()
    control1_to_central = Channel()
    central_to_control2 = Channel()
    control2_to_central = Channel()
    
    control1_to_motor1 = Channel()
    motor1_to_control1 = Channel()

    control2_to_motor2 = Channel()
    motor2_to_control2 = Channel()

    control1_to_control2 = Channel()
    control2_to_control1 = Channel()

    Parallel(motor(motor1_to_control1.writer(), control1_to_motor1.reader()),
             control(control1_to_central.writer(), central_to_control1.reader(),
                     control1_to_motor1.writer(), motor1_to_control1.reader(),
                     control1_to_control2.writer(), control2_to_control1.reader()),
             motor(motor2_to_control2.writer(), control2_to_motor2.reader(), 0.5),
             control(control2_to_central.writer(), central_to_control2.reader(),
                     control2_to_motor2.writer(), motor2_to_control2.reader(),
                     control2_to_control1.writer(), control1_to_control2.reader()),
             central(central_to_control1.writer(), control1_to_central.reader(),
                     central_to_control2.writer(), control2_to_central.reader()))

    shutdown()

if __name__=="__main__":
    main()
