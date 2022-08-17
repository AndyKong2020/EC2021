THRESHOLD = (5, 70, -23, 15, -57, 0)
import time,pyb
from pyb import Pin
from pyb import UART
import sensor, image, time, pyb
import drone
from pid import PID
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.B64X64)
sensor.skip_frames(time = 2000)
clock = time.clock()
pitch_err = 0
roll_err = 0
yaw_err = 0
pitch_output = 360000
roll_output = 360000
yaw_output = 360000
pitch_pid = PID(p=0.1, i=0, d=0)
roll_pid = PID(p=0.1, i=0, d=0)
yaw_pid = PID(p=0.1, i=0, d=0)
BLUE_LED_PIN = 3
pyb.LED(BLUE_LED_PIN).on()
uart = UART(3, 19200)


start = Pin('P6', Pin.IN, Pin.PULL_NONE)
a = 0
b = 0
c = 0
d = 0
f = 0









#111111111111111111111111111111111111111111111111111111111111111
while True:
    a = start.value()
    time.sleep_us(10)
    f += 1
    if a == 1:
        b += 1
    elif a == 0 and b != 0:
        c = c + 1
        d = c / b
        if d < 12 and f > 1000:
            #print(d)
            print(f)
            uart.write("launch")
            time.sleep_ms(20)
            uart.write("launch")
            time.sleep_ms(20)
            uart.write("launch")
            time.sleep_ms(20)
            break
        b = 0
        c = 0
        print(d)

        d = 0
    else:
        c += 1


#222222222222222222222222222222222222222222222222222222222222222222222222222222222
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.B128X128)
sensor.skip_frames(time = 1000)
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 3500, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        #print(cir)
        cirfound = 1
    if cirfound == 1:
        pitch_err = cir.x()-img.width()/2
        roll_err = cir.y()-img.height()/2
        pitch_output = 360000 + 1000*pitch_pid.get_pid(pitch_err,1)
        roll_output = 360000 + 1000*roll_pid.get_pid(roll_err,1)
        if cir[2] < 10:
            break
    #else:
        #pitch_output = 360000
        #roll_output = 360000
    #print(pitch_output,roll_output)
    drone.run(pitch_output,roll_output,360000,0)

#3333333333333333333333333333333333333333333333333333333333333333333333333333333
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.B64X64)
sensor.skip_frames(time = 10)
clock = time.clock()


extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
extra_fb.replace(sensor.snapshot())

speed = 0
speed_err = 0
deltatime = 0

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], roi = (16,16,32,32), robust = True)
    if (line):
        roll_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            yaw_err = line.theta()-180
        else:
            yaw_err = line.theta()
        img.draw_line(line.line(), color = 127)
        #print(roll_err,line.magnitude(),yaw_err)
        if line.magnitude()>8:
            #if -40<b_err<40 and -30<t_err<30:
            roll_output =360000 + 100*roll_pid.get_pid(roll_err,1)
            yaw_output =360000 + 100*yaw_pid.get_pid(yaw_err,1)


        else:
            roll_output = 360000
            yaw_output = 360000


    displacement = extra_fb.find_displacement(img)
    extra_fb.replace(img)

    deltatime = clock.avg()
    #print(deltatime)

    # 没有滤波，偏移结果是嘈杂的，所以我们降低了一些精度。
    sub_pixel_x = int(displacement.x_translation() * 5) / 5.0
    sub_pixel_y = int(displacement.y_translation() * 5) / 5.0

    if(displacement.response() > 0.1): # 低于0.1左右（YMMV），结果只是噪音。
        #print("{0:+f}x {1:+f}y {2} {3} FPS".format(sub_pixel_x, sub_pixel_y,
              #displacement.response(),
              #clock.fps()))
        speed = sub_pixel_y / deltatime
        print(speed)
    #else:
        #print(clock.fps())
        speed_err = 5 - speed
        pitch_output = 360000 + 100*pitch_pid.get_pid(speed_err,1)


    drone.run(pitch_output,roll_output,yaw_output,0)

    dronestep = drone.step()
    if dronestep == True:
        drone.run(pitch_output,roll_output,yaw_output,1)
        break


#444444444444444444444444444444444444444444444444444444444444444444444444444444444
while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 1:
        if cir[0] > 10 and cir[0] < 54:
            break


#555555555555555555555555555555555555555555555555555555555555555555555555555

while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 1:
        if cir[0] > 20 and cir[0] < 44 and cir[1] > 20 and cir[1] < 44:
            uart.write("cirfound")
            break
        pitch_err = cir.x()-img.width()/2
        roll_err = cir.y()-img.height()/2
        pitch_output = 360000 + 100*pitch_pid.get_pid(pitch_err,1)
        roll_output = 360000 + 100*roll_pid.get_pid(roll_err,1)

    print(pitch_output,roll_output)
    drone.run(pitch_output,roll_output,360000,0)
uart.write("cirfound")






#6666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666
rotation_err = 0

while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 1:

        pitch_err = cir.x()-img.width()/2
        roll_err = cir.y()-img.height()/2
        pitch_output = 360000 + 100*pitch_pid.get_pid(pitch_err,1)
        roll_output = 360000 + 100*roll_pid.get_pid(roll_err,1)

    print(pitch_output,roll_output)



    # 如果没有完美的夹具，这个算法很难测试......所以，这是一个让它看起来有效的骗子...
    # 在下面输入一个z_rotation值，你应该看到r的输出等于它。
    if(0):
        expected_rotation = 20.0
        img.rotation_corr(z_rotation=expected_rotation)

    # 如果没有完美的夹具，这个算法很难测试......所以，这是一个让它看起来有效的骗子...
    # 在下面输入一个z_rotation值，你应该看到r的输出等于它。
    if(0):
        expected_zoom = 0.8
        img.rotation_corr(zoom=expected_zoom)

    # 对于此示例，我们从不更新旧图像以测量绝对变化。
    displacement = extra_fb.find_displacement(img, logpolar=True)

    # 没有滤波，偏移结果是嘈杂的，所以我们降低了一些精度。
    rotation_change = int(math.degrees(displacement.rotation()) * 5) / 5.0
    zoom_amount = displacement.scale()

    if(displacement.response() > 0.1): # 低于0.1左右（YMMV），结果只是噪音。
        print("{0:+f}r {1:+f}z {2} {3} FPS".format(rotation_change, zoom_amount, \
              displacement.response(),
              clock.fps()))
        rotation_err = 180 - rotation_change
        yaw_output = 360000 + 100*yaw_pid.get_pid(rotation_err/50,1)
        if rotation_err < 45:
            sensor.snapshot().save("01.jpg")
            break
    else:
        print(clock.fps())

    drone.run(pitch_output,roll_output,yaw_output,0)





#77777777777777777777777777777777777777777777777777777777777777777777777777777777777777777
while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], (8,8,48,48), robust = True)
    if (line):
        roll_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            yaw_err = line.theta()-180
        else:
            yaw_err = line.theta()
        img.draw_line(line.line(), color = 127)
        #print(roll_err,line.magnitude(),yaw_err)
        if line.magnitude()>8:
            #if -40<b_err<40 and -30<t_err<30:
            roll_output =360000 + 100*roll_pid.get_pid(roll_err,1)
            yaw_output =360000 + 100*yaw_pid.get_pid(yaw_err,1)


        else:
            roll_output = 360000
            yaw_output = 360000


    displacement = extra_fb.find_displacement(img)
    extra_fb.replace(img)

    deltatime = clock.avg()
    #print(deltatime)

    # 没有滤波，偏移结果是嘈杂的，所以我们降低了一些精度。
    sub_pixel_x = int(displacement.x_translation() * 5) / 5.0
    sub_pixel_y = int(displacement.y_translation() * 5) / 5.0

    if(displacement.response() > 0.1): # 低于0.1左右（YMMV），结果只是噪音。
        #print("{0:+f}x {1:+f}y {2} {3} FPS".format(sub_pixel_x, sub_pixel_y,
              #displacement.response(),
              #clock.fps()))
        speed = sub_pixel_y / deltatime
        print(speed)
    #else:
        #print(clock.fps())
        speed_err = 5 - speed
        pitch_output = 360000 + 100*pitch_pid.get_pid(speed_err,1)

    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 0:
        break
    drone.run(pitch_output,roll_output,yaw_output,0)






#88888888888888888888888888888888888888888888888888888888888888888888888888888888
while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], (8,8,48,48), robust = True)
    if (line):
        roll_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            yaw_err = line.theta()-180
        else:
            yaw_err = line.theta()
        img.draw_line(line.line(), color = 127)
        #print(roll_err,line.magnitude(),yaw_err)
        if line.magnitude()>8:
            #if -40<b_err<40 and -30<t_err<30:
            roll_output =360000 + 100*roll_pid.get_pid(roll_err,1)
            yaw_output =360000 + 100*yaw_pid.get_pid(yaw_err,1)


        else:
            roll_output = 360000
            yaw_output = 360000


    displacement = extra_fb.find_displacement(img)
    extra_fb.replace(img)

    deltatime = clock.avg()
    #print(deltatime)

    # 没有滤波，偏移结果是嘈杂的，所以我们降低了一些精度。
    sub_pixel_x = int(displacement.x_translation() * 5) / 5.0
    sub_pixel_y = int(displacement.y_translation() * 5) / 5.0

    if(displacement.response() > 0.1): # 低于0.1左右（YMMV），结果只是噪音。
        #print("{0:+f}x {1:+f}y {2} {3} FPS".format(sub_pixel_x, sub_pixel_y,
              #displacement.response(),
              #clock.fps()))
        speed = sub_pixel_y / deltatime
        print(speed)
    #else:
        #print(clock.fps())
        speed_err = 5 - speed
        pitch_output = 360000 + 100*pitch_pid.get_pid(speed_err,1)

    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 1:
        if cir[0] > 10 and cir[0] < 54 and cir[1] > 10 and cir[1] < 54:
            break
    drone.run(pitch_output,roll_output,yaw_output,0)



#99999999999999999999999999999999999999999999999999999999999999999999999999999999999999
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 1:
        if cir[0] > 20 and cir[0] < 44 and cir[1] > 20 and cir[1] < 44:
            uart.write("land")
            break
        pitch_err = cir.x()-img.width()/2
        roll_err = cir.y()-img.height()/2
        pitch_output = 360000 + 100*pitch_pid.get_pid(pitch_err,1)
        roll_output = 360000 + 100*roll_pid.get_pid(roll_err,1)
        if cir[2] < 10:
            break
    print(pitch_output,roll_output)
    drone.run(pitch_output,roll_output,360000,0)
uart.write("land")



#0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    cirfound = 0
    for cir in img.find_circles(threshold = 4000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(cir.x(), cir.y(), cir.r(), color = (255, 0, 0))
        print(cir)
        cirfound = 1
    if cirfound == 1:

        pitch_err = cir.x()-img.width()/2
        roll_err = cir.y()-img.height()/2
        pitch_output = 360000 + 100*pitch_pid.get_pid(pitch_err,1)
        roll_output = 360000 + 100*roll_pid.get_pid(roll_err,1)

    print(pitch_output,roll_output)
    drone.run(pitch_output,roll_output,360000,0)







