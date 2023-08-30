import network
from umqtt.simple import MQTTClient
from machine import Pin, I2C, ADC
import time
import mpu6050
import math
from Kalman import KalmanAngle
import utime

MQTT_SERVER = "192.168.137.101" #Pi 4 with LAPTOP IP
# MQTT_SERVER = "192.168.137.252" #Pi 3
# MQTT_SERVER = "192.168.1.205" #Home wifi

CLIENT_ID = "Glove_controller"
MQTT_TOPIC_0 = b"main"
MQTT_TOPIC_1 = b"main/glove/flex"
MQTT_TOPIC_2 = b"main/glove/angle"

# Laptop mobile hostspot
WIFI_SSID = "MUHAMMAD"
WIFI_PASSWORD = "12345678"


# #Home wifi network
# WIFI_SSID = "NO-SIG-NO"
# WIFI_PASSWORD = "B8Breeze5o9"
# counter = 1

RestrictPitch = True    #Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0

# WiFi connectivity function.
def connectWIFI():
    print("Connecting to "+str(WIFI_SSID)+"...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID,WIFI_PASSWORD)
    while not wlan.isconnected():
        pass
    print ("wifi connected")  
    print (wlan.ifconfig())
    time.sleep(0.1)
  
  
#MQTT callback service function
def sub_cb(topic, msg): 
  print(topic, msg)
  if topic.decode() == MQTT_TOPIC.decode() and msg.decode() == 'ON':
    led.value(0)
    print("LED is ON")
  if topic.decode() == MQTT_TOPIC.decode() and msg.decode() == 'OFF':
    led.value(1)
    print("LED is OFF")
    
def Accl_cal(L_value):
    L_value["AcX"] = round(L_value["AcX"] - 0.0537,2)
    L_value["AcY"] = round(L_value["AcY"] - 0.029,2)
    L_value["AcZ"] = round(L_value["AcZ"] - 0.030,2)
  
    return L_value
def Gyro_cal():
    gyroX_error = gyroY_error = gyroZ_error = 0
    print("Gyro is being calibrated - Place sensor on flat surface")
    for i in range(2000):
       L_value = mpu.get_values()
       gyroX_error  = L_value["GyX"] + gyroX_error
       gyroY_error  = L_value["GyY"] + gyroY_error
       gyroZ_error  = L_value["GyZ"] + gyroZ_error
       time.sleep(0.001)
       if (i % 20 == 0):
           print("Completed: "+str(i/20)+"%",end='\r')
    gyroX_error = gyroX_error / 2000
    gyroY_error = gyroY_error / 2000
    gyroZ_error = gyroZ_error / 2000
    print("Calibration completed and error is as X:"+str(gyroX_error)+"  Y:"+str(gyroY_error)+"  Z:"+str(gyroZ_error))
    time.sleep(1)
    return [gyroX_error,gyroY_error,gyroZ_error]
    
# Flex sensor setup
flex_2 = ADC(Pin(33))
time.sleep(0.1)
print(str(flex_2.read()))


# Connecting to the Local WiFi
connectWIFI()

# Connecting to MQTT Mosquitto Broker server
c = MQTTClient(CLIENT_ID,MQTT_SERVER)
c.set_callback(sub_cb)
c.connect()
c.subscribe(MQTT_TOPIC_0)
print("MQTT connected and ready to receive message")

# Sending message to the Raspberry Pi
c.publish( MQTT_TOPIC_0,"Hello from ESP32")
time.sleep(1)

# MPU initialization and connection
print("MPU 6050 initializing.....")
i2c = I2C(scl=Pin(22), sda=Pin(21))     #initializing the I2C method for ESP32
#i2c = I2C(scl=Pin(5), sda=Pin(4))       #initializing the I2C method for ESP8266
mpu = mpu6050.accel(i2c)

# writing 0x10 to configure acciliration at +-8g in register no 28 (0x1C)
buf = bytearray([0x1C, 0x10])
mpu.write_register_value(buf)
time.sleep(1)

# Calibration of the Gyro
gyro_error = Gyro_cal()
L_value = Accl_cal(mpu.get_values())

# Calculate the roll and pitch angle from the accelegation
Rx = L_value.get("AcX")
Ry = L_value.get("AcY")
Rz = L_value.get("AcZ")
pitch = round(math.atan(-Ry/math.sqrt((Rx**2)+(Rz**2))) * radToDeg,3)
roll = round(math.atan(Rx/math.sqrt((Ry**2)+(Rz**2))) * radToDeg,3)
print("Roll:"+str(roll)+" Pitch:"+str(pitch)+" "+str(Rx)+" "+str(Ry)+" "+str(Rz))


# Gyro rate values
Gx = L_value.get("GyX")-gyro_error[0]
Gy = L_value.get("GyY")-gyro_error[1]
Gz = L_value.get("GyZ")-gyro_error[2]
print("Gx:"+str(Gx)+"  Gy:"+str(Gy)+"  Gz:"+str(Gz))

 
kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;
timer = utime.time()
flag = 0

print(flex_2.read())
time.sleep(1)
while True:
    if(flag >100): #Problem with the connection
        print("There is a problem with the connection")
        flag=0
        continue
    try:
        L_value = Accl_cal(mpu.get_values())
        accX = L_value.get("AcX")
        accY = L_value.get("AcY")
        accZ = L_value.get("AcZ")
        
        gyroX = L_value.get("GyX") - gyro_error[0]
        gyroY = L_value.get("GyY") - gyro_error[1]
        gyroZ = L_value.get("GyZ") - gyro_error[2]
        dt = utime.time() - timer
        timer = utime.time()

        if (RestrictPitch):
            roll = math.atan(accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
            pitch = math.atan(-accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
        else:
            roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
            pitch = math.atan(-accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg

        gyroXRate = gyroX/65.5
        gyroYRate = gyroY/65.5

        if (RestrictPitch):

            if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                kalmanX.setAngle(roll)
                complAngleX = roll
                kalAngleX   = roll
                gyroXAngle  = roll
            else:
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

            if(abs(kalAngleX)>90):
                gyroYRate  = -gyroYRate
                kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
        else:

            if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
                kalmanY.setAngle(pitch)
                complAngleY = pitch
                kalAngleY   = pitch
                gyroYAngle  = pitch
            else:
                kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

            if(abs(kalAngleY)>90):
                gyroXRate  = -gyroXRate
                kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

        #angle = (rate of change of angle) * change in time
        gyroXAngle = gyroXRate * dt
        gyroYAngle = gyroYAngle * dt

        #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
        compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
        compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

        if ((gyroXAngle < -180) or (gyroXAngle > 180)):
            gyroXAngle = kalAngleX
        if ((gyroYAngle < -180) or (gyroYAngle > 180)):
            gyroYAngle = kalAngleY

        #print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
        #print("R:"+str(round(roll,2))+"  GAX:"+str(round(gyroXAngle,2))+"  CAX:"+str(round(compAngleX,2))+"  KAX:"+str(round(kalAngleX,2))+
        #      "  P:"+str(round(pitch,2))+"  GAY:"+str(round(gyroYAngle,2))+"  CAY:"+str(round(compAngleY,2))+"  KAY:"+str(round(kalAngleY,2)))
        msg_1 = str(round(roll+1500)) +","+ str(round(pitch+2500)) +","+ str(round(gyroXAngle+3500)) +","+str(round(gyroYAngle+4500))+","
        msg_2 = str(round(kalAngleX+5500))+","+str(round(kalAngleY+6500))+","+str(round(compAngleX+7500))+","+str(round(compAngleY+8500))+","
        msg_3 = str(flex_2.read()+9500)
        print(msg_1 + msg_2 + msg_3,end='\r')
        c.publish(MQTT_TOPIC_2, msg_1 + msg_2 + msg_3)
        time.sleep(0.5)

    except Exception as exc:
        flag += 1
