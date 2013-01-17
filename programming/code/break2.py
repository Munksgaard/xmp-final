# ; -*- mode: Python; eval: (setq python-command "python2"); -*-

import sys
import time
from pycsp.parallel import *

acceleration = 0.1
acc_threshold = 20
epsilon = 0.1

@process
def control(from_central, to_central, from_motor, to_motor):
    while True:
        target_speed = from_central()
        reported_speed = from_motor()
        reported_speed = go_to_speed(from_motor,
                                     to_motor,
                                     reported_speed,
                                     target_speed)
        print "Successfully reached speed " + str(reported_speed)

def go_to_speed(from_motor, to_motor, reported_speed, target_speed):
    to_motor(target_speed)
    while abs(reported_speed - target_speed) > epsilon:
        new_reported_speed = from_motor()
        #print "new_reported_speed: " + str(new_reported_speed)
        if abs(new_reported_speed - reported_speed) > 1:
            #print "Must be spinning! ns: " + str(new_reported_speed) + ", os: " + str(reported_speed) + ", ts: " + str(target_speed)
            reported_speed = go_to_speed(from_motor,
                                         to_motor,
                                         reported_speed,
                                         (target_speed - reported_speed)
                                           / 2 + reported_speed)
            to_motor(target_speed)
        else:
            reported_speed = new_reported_speed
    return reported_speed

@process
def motor(from_control, to_control):
    actual_speed = 0
    observed_speed = 0
    target_speed = 0
    timestamp = time.time()
    while True:
        chan_end, msg = AltSelect(InputGuard(from_control),
                                  OutputGuard(to_control, observed_speed),
                                  SkipGuard())

        new_timestamp = time.time()

        if chan_end == from_control:
            target_speed = msg
        elif abs(target_speed - observed_speed) > epsilon:
            if abs(target_speed - actual_speed) > acc_threshold:
                 # Not enough traction, we are spinning
                observed_speed = target_speed
            elif target_speed < actual_speed:
                observed_speed = actual_speed = max(actual_speed - (new_timestamp-timestamp)
                                                      * (1 / acceleration),
                                                    0)
            elif target_speed > actual_speed:
                observed_speed = actual_speed = min(actual_speed + (new_timestamp-timestamp)
                                                      * (1 / acceleration),
                                                    100)

        timestamp = new_timestamp
                            
@process
def central(from_control, to_control):
    to_control(100)

def main():
    central_to_control = Channel()
    control_to_central = Channel()
    control_to_motor = Channel()
    motor_to_control = Channel()

    Parallel(central(control_to_central.reader(), central_to_control.writer()),
             control(central_to_control.reader(),
                     control_to_central.writer(),
                     motor_to_control.reader(),
                     control_to_motor.writer()),
             motor(control_to_motor.reader(),
                   motor_to_control.writer()))

    shutdown()

if __name__ == "__main__":
    main()
