from pyb import Pin, Timer
import time
from pyb import UART

uart = UART(3, 19200)

a = Pin('P6') # P7 has TIM4, CH1

tim = Timer(2, freq=50)
ch1 = tim.channel(1, Timer.PWM, pin=a)
filt = []

def run(pitch, roll, yaw, step):
    ctrl = (pitch, roll, yaw, step)
    uart.write(str(ctrl))

def get_mid(data):
    data.sort()
    half = len(data) // 2
    return (data[half] + data[~half]) / 2

def acc(acc):

    if len(filt) > 8:
        mid = get_mid([filt[0],filt[1],filt[2],filt[3],filt[4],filt[5],filt[6],filt[7],filt[8]])
        if (acc-mid>15000) or (mid-acc>15000):
            acc = flit[8]
        del filt[0]
        filt.append(acc)
    if acc > 360000:
        acc = 360000
    elif acc < 330000:
        acc = 330000
    print(acc)
    ch1.pulse_width(acc)


