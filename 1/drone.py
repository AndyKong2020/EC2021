from pyb import Pin, Timer
import time
from pyb import UART

uart = UART(3, 19200)

p = Pin('P7') # P7 has TIM4, CH1
r = Pin('P8')
y = Pin('P9')

tim = Timer(4, freq=50)
ch1 = tim.channel(1, Timer.PWM, pin=p)
ch2 = tim.channel(2, Timer.PWM, pin=r)
ch3 = tim.channel(3, Timer.PWM, pin=y)

def run(pitch, roll, yaw, step):
    ctrl = [pitch, roll, yaw, step]
    #pitch = ctrl[0]
    #roll = ctrl[1]
    #yaw = ctrl[2]
    #step = ctrl[3]

    if step == 1:
        a = str(uart.read())
        print(a)
        if a != "None":
            b = str(a).strip("b'(")
            c = b.rstrip(")")

            ctrl = c.split(",")
            pitch = int(10000*float(ctrl[0]))
            roll = int(10000*float(ctrl[1]))
            yaw = int(10000*float(ctrl[2]))
            step = 1
            ctrl = [pitch, roll, yaw, step]
    pitch = int(ctrl[0])
    roll = int(ctrl[1])
    yaw = int(ctrl[2])
    ch1.pulse_width(pitch)
    ch2.pulse_width(roll)
    ch3.pulse_width(yaw)
    print(ctrl)
def step():
    a = str(uart.read())
    print(a)
    if a != "None":
        b = str(a).strip("b'(")
        c = b.rstrip(")")

        ctrl = c.split(",")
        return ctrl[3]
    else:
        return
        #return a
