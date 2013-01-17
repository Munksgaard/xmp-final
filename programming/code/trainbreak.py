# ; -*- mode: Python; eval: (setq python-command "python2"); -*-

import sys
from pycsp.parallel import *

acc_threshold = 20

class NewTargetSpeedException(Exception):
    def __init__(self, target_speed, observed_speed):
        self.target_speed = target_speed
        self.observed_speed = observed_speed

    def newTarget(self):
        return self.target_speed

    def observedSpeed(self):
        return self.observed_speed

@process
def control(control_in, control_out, other_in, other_out, motor_in, motor_out):
    observed_speed = 0
    flag = False

    while True:
        if not flag:
            target_speed = control_in()
        else:
            flag = False

        try:
            observed_speed = go_to_speed(control_in, 
                                         other_in, 
                                         other_out, 
                                         motor_in, 
                                         motor_out, 
                                         observed_speed, 
                                         target_speed)
            print "Successfully reached speed " + str(observed_speed)
        except NewTargetSpeedException as e:
            target_speed = e.newTarget()
            observed_speed = e.observedSpeed()
            print "New target speed! " + str(target_speed)
            flag = True

def go_to_speed(control_in, 
                other_in, 
                other_out, 
                motor_in, 
                motor_out, 
                observed_speed, 
                target_speed):
    motor_out(target_speed)
    print "new go_to_speed. os: " + str(observed_speed) + ", ts: " + str(target_speed)

    while observed_speed != target_speed:
        #new_speed = motor_in()
        #print str(control_in) + "observed_speed: " + str(observed_speed)
        #print str(control_in) + "target_speed: " + str(target_speed)
        ch_end, msg = AltSelect(InputGuard(control_in),
                                InputGuard(motor_in))
        if ch_end == control_in:
            raise NewTargetSpeedException(msg, observed_speed)
        else:
            new_speed = msg
            other_speed = exchange_speeds(new_speed, other_in, other_out)
            if (new_speed > other_speed and target_speed >= new_speed) or (new_speed < other_speed and target_speed <= new_speed):
                print "Out of sync! Waiting! ns: " + str(new_speed) + ", os: " + str(other_speed) + ", ts: " + str(target_speed)
                motor_out(new_speed)
            else: 
                #print str(control_in) + "new_speed: " + str(new_speed)
                if abs(new_speed - observed_speed) > 1:
                    print "Must be spinning! ns: " + str(new_speed) + ", os: " + str(observed_speed) + ", ts: " + str(target_speed)
                    observed_speed = go_to_speed(control_in,
                                                 other_in,
                                                 other_out,
                                                 motor_in,
                                                 motor_out,
                                                 observed_speed,
                                                 (target_speed - observed_speed)
                                                   / 2 + observed_speed)
                    motor_out(target_speed)
                else:
                    observed_speed = new_speed
    return observed_speed

def exchange_speeds(observed_speed, chan_in, chan_out):
    ch_end, msg = AltSelect(InputGuard(chan_in),
                            OutputGuard(chan_out, observed_speed))
    if ch_end == chan_in:
        chan_out(observed_speed)
        return msg
    else:
        return chan_in()

@process
def motor(chan_in, chan_out, delay=0.1):
    wheel_speed = 0
    observed_speed = 0
    target_speed = 0

    while True:
        ch_end, msg = AltSelect(InputGuard(chan_in),
                                TimeoutGuard(seconds=delay))
        if ch_end == chan_in:
            target_speed = msg

        if abs(target_speed - wheel_speed) > acc_threshold:
            print "Spinning! ws = " + str(wheel_speed) + ", os = " + str(observed_speed) + ", ts = " + str(target_speed)
            observed_speed = target_speed
        elif target_speed < wheel_speed:
            observed_speed = wheel_speed = wheel_speed - 1
        elif target_speed > wheel_speed:
            observed_speed = wheel_speed = wheel_speed + 1
        else:
            pass

        AltSelect(OutputGuard(chan_out, msg=observed_speed),
                  TimeoutGuard(seconds=0.01))


@process
def central(control1_in, control1_out, control2_in, control2_out):
    control1_out(100)
    control2_out(100)
    #retire(chan_out)

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

    Parallel(motor(control1_to_motor1.reader(), motor1_to_control1.writer()),
             control(central_to_control1.reader(),
                     control1_to_central.writer(),
                     control2_to_control1.reader(),
                     control1_to_control2.writer(),
                     motor1_to_control1.reader(),
                     control1_to_motor1.writer()),
             motor(control2_to_motor2.reader(), motor2_to_control2.writer()),
             control(central_to_control2.reader(),
                     control2_to_central.writer(),
                     control1_to_control2.reader(),
                     control2_to_control1.writer(),
                     motor2_to_control2.reader(),
                     control2_to_motor2.writer()),
             central(control1_to_central.reader(),
                     central_to_control1.writer(),
                     control2_to_central.reader(),
                     central_to_control2.writer()))

    shutdown()

if __name__=="__main__":
    main()
