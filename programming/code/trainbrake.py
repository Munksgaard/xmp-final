# ; -*- mode: Python; eval: (setq python-command "python2"); -*-

import sys
import time
from pycsp.greenlets import *

acc_threshold = 20
epsilon = 0
debug = True


class NewSpeedException(Exception):
    def __init__(self, obs_speed, new_target):
        self.obs_speed = obs_speed
        self.new_target = new_target

    def getObsSpeed(self):
        return self.obs_speed

    def getNewTarget(self):
        return self.new_target


@process
def motor(num, to_control, from_control, skew=1):
    last_speed = 0

    while True:
        target_speed = from_control()
        if abs(target_speed - last_speed) > acc_threshold:
            # spin
            to_control(target_speed)
        elif target_speed < last_speed - epsilon:
            # decelerate
            last_speed -= skew
            to_control(last_speed)
        elif target_speed > last_speed + epsilon:
            # accelerate
            last_speed += skew
            to_control(last_speed)
        else:
            last_speed = target_speed
            to_control(target_speed)
        

@process
def control(num,
            to_central, 
            from_central,
            to_motor, 
            from_motor, 
            to_other, 
            from_other):
    observed_speed = 0

    while True:
        target_speed = from_central()

        while True:
            try:
                observed_speed = go_to_speed(num,
                                             from_central,
                                             to_motor, 
                                             from_motor,
                                             to_other,
                                             from_other, 
                                             observed_speed,
                                             target_speed)
                c = None
                while c != to_central:
                    c, msg = AltSelect(OutputGuard(to_central, num + 
                                                   ": Successfully reached speed "
                                                   + str(observed_speed)),
                                       OutputGuard(to_other, observed_speed),
                                       InputGuard(from_other))
                break
            except NewSpeedException as e:
                observed_speed = e.getObsSpeed()
                target_speed = e.getNewTarget()

def go_to_speed(num, from_central, 
                to_motor, 
                from_motor, 
                to_other, 
                from_other, 
                observed_speed, 
                target_speed):
    while observed_speed != target_speed:
        g, msg = AltSelect(InputGuard(from_central),
                           OutputGuard(to_motor, target_speed))
        if g == from_central:
            if debug: print num + ": New target!" + str(msg)
            raise NewSpeedException(observed_speed, msg)

        new_obs_speed = from_motor()
        
        if abs(new_obs_speed - observed_speed) > 1:
            if debug: print num + ": spinning/blocking " + str(new_obs_speed)
            observed_speed = go_to_speed(num, 
                                         from_central, 
                                         to_motor, 
                                         from_motor, 
                                         to_other, 
                                         from_other, 
                                         observed_speed, 
                                         (target_speed - observed_speed) 
                                           / 2 + observed_speed)
        else:
            if debug: print num + ": new_obs_speed: " + str(new_obs_speed)
            observed_speed = new_obs_speed
            
        other_speed = exchange_speeds(observed_speed, to_other, from_other)
        if other_speed != observed_speed: 
            if debug:print (num + ": Out of sync! obs: " + str(observed_speed) 
                            + ", os: " + str(other_speed))
            if (observed_speed < other_speed <= target_speed or
                target_speed <= other_speed < observed_speed):
                if debug: print num + ": We're behind, going ahead!"
            else:
                if debug: print num + ": We're ahead! Waiting!"
                while True:
                    g, msg = AltSelect(InputGuard(from_central),
                                       OutputGuard(to_motor, observed_speed))
                    if g == from_central:
                        if debug: print num + ": New target (while waiting)"
                        raise NewSpeedException(observed_speed, msg)
                    observed_speed = from_motor()
                    other_speed = exchange_speeds(observed_speed,
                                                  to_other,
                                                  from_other)
                    if ((observed_speed >= other_speed and 
                        observed_speed <= target_speed) or 
                        (target_speed <= observed_speed and 
                         other_speed >= observed_speed)):
                        break

    return observed_speed

def exchange_speeds(observed_speed, to_other, from_other):
    ch_end, msg = AltSelect(InputGuard(from_other),
                            OutputGuard(to_other, 
                                        observed_speed))
    if ch_end == from_other:
        to_other(observed_speed)
        return msg
    else:
        return from_other()

def spawnWheelset(num, skew1=1, skew2=1):
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

    Spawn(motor(str(num) + "l", 
                motor1_to_control1.writer( ), 
                control1_to_motor1.reader(), 
                skew1),
          control(str(num) + "l",
                  control1_to_central.writer(), 
                  central_to_control1.reader(),
                  control1_to_motor1.writer(), 
                  motor1_to_control1.reader(),
                  control1_to_control2.writer(), 
                  control2_to_control1.reader()),
          motor(str(num) +"r", 
                motor2_to_control2.writer(), 
                control2_to_motor2.reader(), 
                skew2),
          control(str(num) + "r",
                  control2_to_central.writer(), 
                  central_to_control2.reader(),
                  control2_to_motor2.writer(), 
                  motor2_to_control2.reader(),
                  control2_to_control1.writer(),
                  control1_to_control2.reader()))

    return [(central_to_control1.writer(), 
             control1_to_central.reader()),
            (central_to_control2.writer(), 
             control2_to_central.reader())]

def spawnWheelsets(n, skew1=1, skew2=1):
    chans = []
    for i in range(n):
        chans += spawnWheelset(i, skew1, skew2)

    return chans

def test0():
    chans = spawnWheelsets(10)

    for (to, fro) in chans:
        to(15)

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        retire(to, fro)

    shutdown()

def test1():
    chans = spawnWheelsets(10)

    for (to, fro) in chans:
        to(15)

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        to(5)

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        retire(to, fro)

    shutdown()

def test2():
    chans = spawnWheelsets(10)

    for (to, fro) in chans:
        to(100)

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        to(30)

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        retire(to, fro)

    shutdown()

def test3():
    chans = spawnWheelsets(10)

    for (to, fro) in chans:
        to(100)

    for (to, fro) in chans:
        c = None
        while c != to:
            c, msg = AltSelect(OutputGuard(to, 15),
                               InputGuard(fro))

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        retire(to, fro)

    shutdown()

def test4():
    global epsilon
    old_epsilon = epsilon
    epsilon = 0.1

    chans = spawnWheelsets(10, 0.5)

    for (to, fro) in chans:
        to(100)

    for (to, fro) in chans:
        print fro()

    for (to, fro) in chans:
        retire(to, fro)

    shutdown()

    epsilon = old_epsilon

def main():
    pass

if __name__=="__main__":
    main()
