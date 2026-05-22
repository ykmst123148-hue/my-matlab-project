#!/usr/bin/env python3

import serial
import struct
import time
import RPi.GPIO as GPIO
import csv
import random

class Sabo:
    def __init__(self, ser, ID, pin, **kwargs):
        self.ser = ser
        self.ID = ID
        self.pin = pin
        self.targetangle = 0
        self.currentangle = 0
        self.I = kwargs.pop("I", 0.01)
        self.D = kwargs.pop("D", 0.5)
        self.K = kwargs.pop("K", 1.0)
        self.Gn = ((self.I + self.D*T + (self.K*T*T)))
        self.G1 = (T*T)/self.Gn
        self.G2 = (((2*self.I) + (self.D*T))/self.Gn)
        self.G3 = -self.I/self.Gn
        self.o1 = 0
        self.o2 = 0

    def transmission(self, data):
        cmds = data

        checksum = 0x00
        for cmd in cmds[2:]:
            checksum ^= cmd
        cmds.append(checksum)

        data = bytes(cmds)
        GPIO.output(self.pin, GPIO.LOW)
        self.ser.write(data)
        #self.ser.flush()
        time.sleep(0.002)
        GPIO.output(self.pin, GPIO.HIGH)

    def send1(self, Flag, Address, data):
        cmds = [0xFA, 0xAF, self.ID, Flag, Address, 0x01, 0x01, data]
        self.transmission(cmds)

    def send2(self, Flag, Address, data):
        cmds = [0xFA, 0xAF, self.ID, Flag, Address, 0x02, 0x01, 0x00FF & data, 0x00FF & (data >> 8)]
        self.transmission(cmds)

    def Return(self, Flag, Address, Len):
        cmds = [0xFA, 0xAF, self.ID, Flag, Address, Len, 0x00]
        self.transmission(cmds)

    def toruque(self, data):
        cmds = [0xFA, 0xAF, self.ID, 0x00, 0x24, 0x01, 0x01, data]
        self.transmission(cmds)

    def move(self, Angle, Speed):
        cmds = [0xFA, 0xAF, self.ID, 0x00, 0x1E, 0x04, 0x01, 0x00FF & Angle, 0x00FF & (Angle >> 8), 0x00FF & Speed, 0x00FF & (Speed >> 8)]
        self.transmission(cmds)
        self.targetangle = Angle
        time.sleep(0.001)

    def write(self):
        cmds = [0xFA, 0xAF, self.ID, 0x40, 0xFF, 0x00, 0x00]
        self.transmission(cmds)

    def reboot(self):
        cmds = [0xFA, 0xAF, self.ID, 0x20, 0xFF, 0x00, 0x00]
        self.transmission(cmds)

    def readangle(self):
        cmds = [0xFA, 0xAF, self.ID, 0x0F, 0x2A, 0x02, 0x00]
        self.transmission(cmds)
        time.sleep(0.001)

        str1 = self.ser.read_until(b'\xfd\xdf')
        data = self.ser.read(7)
        a = []
        for i in data:
            a.append(i)
        if(len(a) > 6):
            b = a[6] << 8 ^ a[5]
            c = (int(b ^ 0xffff) * -1)-1 if (b & 0x8000) else int(b)
            print(f"angle:{c/10}[deg]")
        else:
            pass

    def readtoruque(self):
        cmds = [0xFA, 0xAF, self.ID, 0x0F, 0x30, 0x02, 0x00]
        self.transmission(cmds)
        time.sleep(0.001)

        str1 = self.ser.read_until(b'\xfd\xdf')
        data = self.ser.read(7)
        a = []
        for i in data:
            a.append(i)
        if(len(a) > 6):
            b = a[6] << 8 ^ a[5]
            c = (int(b ^ 0xffff) * -1)-1 if (b & 0x8000) else int(b)
            print(f"load:{c}[mA]")
        else:
            pass

    def readtanda(self):
        cmds = [0xFA, 0xAF, self.ID, 0x0F, 0x2A, 0x08, 0x00]
        self.transmission(cmds)
        time.sleep(0.001)

        str1 = self.ser.read_until(b'\xfd\xdf')
        data = self.ser.read(14)
        a = []
        for i in data:
            a.append(i)
        if(len(a) > 13):
            b1 = a[6] << 8 ^ a[5]
            b2 = a[12] << 8 ^ a[11]
            c1 = (int(b1 ^ 0xffff) * -1)-1 if (b1 & 0x8000) else int(b1)
            c2 = (int(b2 ^ 0xffff) * -1)-1 if (b2 & 0x8000) else int(b2)
            #print(f"angle:{c1/10} toruque:{c2}")
            data = [c1/10, c2]
            return data

    def com(self):
        cmds = [0xFA, 0xAF, self.ID, 0x0F, 0x2A, 0x08, 0x00]
        self.transmission(cmds)
        #time.sleep(0.001)

        str1 = self.ser.read_until(b'\xfd\xdf')
        data = self.ser.read(14)
        a = []
        for i in data:
            a.append(i)

    
        if(len(a) > 13):
            b1 = a[6] << 8 ^ a[5]
            b2 = a[12] << 8 ^ a[11]
            c1 = (int(b1 ^ 0xffff) * -1)-1 if (b1 & 0x8000) else int(b1)
            c2 = (int(b2 ^ 0xffff) * -1)-1 if (b2 & 0x8000) else int(b2)
            if(c2 > 10):
                if((self.currentangle - c1) > 0):
                    t_ex = k*c2
                    d_theta = self.G1*t_ex + self.G2*self.o1 + self.G3*self.o2
                    dt = round(d_theta*180/3.14, 1)
                    self.o2 = self.o1
                    self.o1 = d_theta
                    print(dt)
                    angle = self.targetangle
                    self.move(angle-int(dt*10), MOVE_SPEED)
                    self.targetangle = angle
                else:
                    t_ex = k*c2
                    d_theta = self.G1*t_ex + self.G2*self.o1 + self.G3*self.o2
                    dt = round(d_theta*180/3.14, 1)
                    self.o2 = self.o1
                    self.o1 = d_theta
                    print(dt)
                    angle = self.targetangle
                    self.move(angle+int(dt*10), MOVE_SPEED)
                    self.targetangle = angle
                self.currentangle = c1
            else:
                self.move(self.targetangle, MOVE_SPEED)
        else:
            #pass
            print(555)


#--------------------------------------------------------------
#最初にいるやつ
ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=0.01)
pin = 22
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin,GPIO.OUT)
time.sleep(0.5)
T = 0.00761 
k = 0.00215 
MOVE_SPEED = 10
#--------------------------------------------------------------

s1 = Sabo(ser, 1, pin, I=0.01, D=0.5, K=1.0)
s1.toruque(1)
time.sleep(0.1)
s1.move(0, 1)
time.sleep(1)

"""
csv_filename = 'angle_data.csv'
csv_file = open(csv_filename, 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['time', 'angle'])
main_start_time = time.time()
"""

#"""
csv_filename = 'com_data.csv'
csv_file = open(csv_filename, 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['time', 'angle'])
main_start_time = time.time()
#"""

#""""
try:
    while True:
        #s1.readtoruque()
        #s1.readangle()
        #s1.direction()
        s1.com()

        elapsed_time = time.time() - main_start_time
        current_angle_in_degrees = s1.currentangle / 10.0
        writer.writerow([elapsed_time, current_angle_in_degrees])

        #time.sleep(0.001)
except KeyboardInterrupt:
    pass
#"""

"""
n = 0
try:
    while True:
        if(n == 0):
            s1.move(400, 100)
            start = time.time()
            while((time.time() - start) < 1):
                s1.com()
                #追加(2025/06/16)
                elapsed_time = time.time() - main_start_time
                current_angle = s1.currentangle/10
                writer.writerow([elapsed_time, current_angle])
                #追加ここまで
            n = 1
        else:
            s1.move(-400, 100)
            start = time.time()
            while((time.time() - start) < 1):
                s1.com()
                elapsed_time = time.time() - main_start_time
                current_angle = s1.currentangle/10
                writer.writerow([elapsed_time, current_angle])
            n = 0
except KeyboardInterrupt:
    pass
"""

s1.toruque(0)
#--------------------------------------------------------------
#最後にいるやつ
csv_file.close()
ser.close()
GPIO.cleanup(pin)
#-------------------------------------------------------------