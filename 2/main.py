# code block
#openmv利用超声波测距
import time,utime,pyb,sensor,image
from pyb import Pin
from pid import PID
import drone
from pyb import UART

uart = UART(3, 19200)


height_pid = PID(p=0.6, i=0, d=0.1)

wave_echo_pin = Pin('P0', Pin.IN, Pin.PULL_NONE)
wave_trig_pin = Pin('P1', Pin.OUT_PP, Pin.PULL_DOWN)

wave_distance = 0
tim_counter = 0
flag_wave = 0
height_err = 0
height_output = 0

BLUE_LED_PIN = 3



sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_vflip(True)
sensor.set_hmirror(True)
clock = time.clock() # Tracks FPS.

blue_threshold   = (44, 100, -71, 12, -128, 28)
#dist_threshold = 1000
pitch_pid = PID(p=0.5, i=1, imax=100)
roll_pid = PID(p=0.05, i=0.1, imax=50)
yaw_pid = PID(p=0.05, i=0.1, imax=50)

pitch_err = 0
pitch_output = 0
roll_err = 0
roll_output = 0
yaw_err = 0
yaw_output = 0


def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

#超声波启动
def wave_start():
    #wave_trig_pin.value(0)
    #utime.sleep_us(2)
    #wave_trig_pin.value(1)
    #utime.sleep_us(13)
    #wave_trig_pin.value(0)
    wave_trig_pin.value( 1 )
    utime.sleep_us( 15 )
    wave_trig_pin.value( 0 )

#超声波距离计算
def wave_distance_calculation1():
#全局变量声明
    global tim_counter
#频率f为0.2MHZ 高电平时间t=计数值1/f
    wave_distance = tim_counter*5*0.0105
#输出最终的测量距离（单位cm）
    height_err = 80 - wave_distance
    #print(hei ght_err)
    height_output = height_pid.get_pid(height_err,1)
    #print(height_output)

    acc = int(342000 + 1000*height_output)
    print(acc)
    drone.acc(acc)
    print('wave_distance',wave_distance)

#超声波数据处理
def wave_distance_process1():
    global flag_wave
    if(flag_wave == 0):
        wave_start()

        return
    if(flag_wave == 2):
        wave_distance_calculation1()
        return True




def wave_distance_calculation2():
#全局变量声明
    global tim_counter
#频率f为0.2MHZ 高电平时间t=计数值1/f
    wave_distance = tim_counter*5*0.0105
#输出最终的测量距离（单位cm）
    height_err = 40 - wave_distance
    #print(hei ght_err)
    height_output = height_pid.get_pid(height_err,1)
    #print(height_output)

    acc = int(310000 + 1000*height_output)
    print(acc)
    drone.acc(acc)
    #print('wave_distance',wave_distance)

#超声波数据处理
def wave_distance_process2():
    global flag_wave
    if(flag_wave == 0):
        wave_start()

        return
    if(flag_wave == 2):
        wave_distance_calculation2()
        return True






def wave_distance_calculation3():
#全局变量声明
    global tim_counter
#频率f为0.2MHZ 高电平时间t=计数值1/f
    wave_distance = tim_counter*5*0.0105
#输出最终的测量距离（单位cm）
    height_err = 7 - wave_distance
    #print(hei ght_err)
    height_output = height_pid.get_pid(height_err,1)
    #print(height_output)
    if wave_distance < 7:
        acc = 0
    else:
        acc = int(330000 + 1000*height_output)
        print(acc)
        drone.acc(acc)
    #print('wave_distance',wave_distance)

#超声波数据处理
def wave_distance_process3():
    global flag_wave
    if(flag_wave == 0):
        wave_start()

        return
    if(flag_wave == 2):
        wave_distance_calculation3()
        return True



#配置定时器
tim =pyb.Timer(4, prescaler=720, period=65535) #相当于freq=0.2M

#外部中断配置
def callback(line):
    #print("line =", line)
    global flag_wave,tim_counter
#上升沿触发处理
    if(wave_echo_pin.value()):
        tim.init(prescaler=720, period=65535)
        flag_wave = 1
#下降沿
    else:
        tim.deinit()
        tim_counter = tim.counter()
        tim.counter(0)
        extint.disable()
        flag_wave = 2
#中断配置
extint = pyb.ExtInt(wave_echo_pin, pyb.ExtInt.IRQ_RISING_FALLING, pyb.Pin.PULL_DOWN, callback)

ok = 0
step = 0




while True:
    launch = str(uart.read())
    print(launch)
    if launch == "b'launch'":
        print(launch)
        break
    time.sleep_ms(20)






while True:
    #print(1)
    pyb.LED(BLUE_LED_PIN).on()
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([blue_threshold],pixels_threshold = 2800 )
    if blobs:
        max_blob = find_max(blobs)

        roll_err = max_blob[5]-img.width()/2 - 5

        yaw_err = max_blob[5]-img.width()/2

        pitch_err =90 - max_blob[2]

        img.draw_rectangle(max_blob[0:4]) # rect
        img.draw_cross(max_blob[5], max_blob[6])
        #print("roll_err: ", roll_err)
        #print("yaw_err: ", yaw_err)
        #print("pitch_err: ", pitch_err)
        roll_output=roll_pid.get_pid(roll_err,1)
        yaw_output=yaw_pid.get_pid(yaw_err,1)
        pitch_output=pitch_pid.get_pid(pitch_err,1)
        step = 1

        print("roll_output: ", roll_output)
        print("yaw_output: ", yaw_output)
        print("pitch_output: ", pitch_output)
        drone.run(pitch_output,roll_output,yaw_output,step)

    extint.enable()
    while(True):


        ok = wave_distance_process1()
        #print(ok)






        if ok:
            ok = 0
            #print("2")
            break
        #time.sleep_ms(10)


    wave_echo_pin = Pin('P0', Pin.IN, Pin.PULL_NONE)
    wave_trig_pin = Pin('P1', Pin.OUT_PP, Pin.PULL_DOWN)

    wave_distance = 0
    tim_counter = 0
    flag_wave = 0
    extint.disable()


    cirfound = str(uart.read())

    if cirfound == "b'cirfound'":
        break


while True:
    extint.enable()
    while(True):


        ok = wave_distance_process2()
        #print(ok)






        if ok:
            ok = 0
            #print("2")
            break
        #time.sleep_ms(10)


    wave_echo_pin = Pin('P0', Pin.IN, Pin.PULL_NONE)
    wave_trig_pin = Pin('P1', Pin.OUT_PP, Pin.PULL_DOWN)

    wave_distance = 0
    tim_counter = 0
    flag_wave = 0
    extint.disable()


    land = str(uart.read())

    if land == "b'land'":
        break





while True:
    extint.enable()
    while(True):


        ok = wave_distance_process3()
        #print(ok)






        if ok:
            ok = 0
            #print("2")
            break
        #time.sleep_ms(10)


    wave_echo_pin = Pin('P0', Pin.IN, Pin.PULL_NONE)
    wave_trig_pin = Pin('P1', Pin.OUT_PP, Pin.PULL_DOWN)

    wave_distance = 0
    tim_counter = 0
    flag_wave = 0
    extint.disable()







