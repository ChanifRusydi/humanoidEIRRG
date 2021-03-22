# UPDATE PERTAMA: 10-12-2020
# Update 2 Fungsi di bawah ini:
# 1. rot2RPY(r)
# 2. RPY2R(rpy)
# mengacu ke tutorial Youtube: Christopher Lum!
# dan men-delete fungsi yang dipakai sebelumnya
# fungnya ada di slide_si sebelum19_baru.py !
# https://www.youtube.com/watch?v=GJBc6z6p0KQ

# UPDATE KEDUA: 13-12-2020
# Validasi fungsi untuk Forward Kinematics dan Inverse Kinematics
# dengan 6 data INPUT REAL, RESULT: OK !
# step verifikasi:
# FK, input: X, output: Y
# IK, input: Y, output: X
# artinya: FK dan IK sudah valid, insya Allah.

# UPDATE KETIGA: 15-12-2020
# Penambahan proses sorting servo [sorting_servo(goalPoss)]
# fungsi ini akan mengubah input goalPoss menjadi arrServoKa dan arrServoKi
# syaratnya servo ID 7,8,9,10,11,12,13,14,15,16,17,18 harus ada dalam GoalPoss [PENTING!]
# sehingga bisa dilakukan perhitungan Inverse Kinematics dan Forward Kinematics !
# source: https://www.afternerd.com/blog/python-sort-list/

# UPDATE KEEMPAT: 16-12-2020
# bikin kalibrasi otomatis dari python (kalibrasi = 1[ON], kalibrasi = 0 [OFF])

# UPDATE KELIMA: 22-12-2020
# gabungin program IK dan FK
# adjust kaki dan tangan sebesar alfa dan beta
# Gabungin IK,FK,Fuzzy sama Kalibrasi ke 1 program

# UPDATE KEENAM: 23-12-2020
# cek ke robot langsung!
# orientasi kaki kanan bawah (pitch??) kebalik ! [PROGRESS CHECKING !]

# To DO Next: 
# [Khusu Buat Paper !] PERSIAPAN UNTUK FUZZY ROLL ! buat cadangan kalo FUZZY PITCH doang gak cukup!
# PENGUJIAN
# Alhamdulillah, ada Allah :)


import serial
import os
import time
from time import sleep
from threading import *
from fractions import Fraction as F 
import numpy as np
import math
from struct import *
import array as arr
import sys
import numpy.linalg as lin

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36

# Control table address for Dynamixel MX
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36

# Data Byte Length
LEN_AX_GOAL_POSITION       = 4
LEN_AX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
#BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
BAUDRATE                    = 57600           # Dynamixel default baudrate : 57600
#DEVICENAME                  = "/dev/ttyUSB0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 32                # Dynamixel moving status threshold AX
DXL2_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold MX

# parameter gerakan
walkready = 0
fml2_6_ = 0     # jalan
fmr2_6_ = 0     # jalan
# .....

# inisialisasi parameter kalibrasi MPU6050
index = 0
goalPoss = 512
baca_sensor = 0
mulai_baca = 0
full_body = 1
verMX = 1       # versi MX mode default
verMX2= 0       # versi MX mode baru (tangan ke depan)
ver2 = 0        
verAX2 = 0      # versi AX mode baru
# parameter miring depan-belakang hip
y = 20
#============ speed jalan ==================================
speed = 170
speed_awal_jalan = 511
speed_akhir_jalan = 255
#============= speed jalan samping =========================
speed_jalan_samping = 255
speed_awal_samping_kanan = 255
speed_akhir_samping_kanan = 255
speed_awal_samping_kiri = 255
speed_akhir_samping_kiri = 255
#============= speed muter =================================
speed_muter_kanan = 255
speed_muter_kiri = 150

# ============ inisialisasi Inverse Kinematics =============
class LINK:

    def __init__(self,name,m,sister,child,b,a,q):
        self.name = name
        self.m = m
        self.sister = sister
        self.child = child
        self.b = b
        self.a = a
        self.q = q

class foot:

    def __init__(self):
        pass


# Inisialisasi Parameter Inverse Kinematics
UX = np.transpose([[1,0,0]])
UY = np.transpose([[0,1,0]])
UZ = np.transpose([[0,0,1]])
pi = math.pi

uLINK = []
uLINK.append(LINK("KOSONG",-1,-1,-1,-1,-1,-1))              # 0, GAK DIPAKE! biar bisa mulai dari 1 aja! mirip matlab

uLINK.append(LINK("BODY",10,0,2,np.transpose([[0,0,0.7]]),UZ,0))        # 1
uLINK.append(LINK("RLEG_J0",5,8,3,np.transpose([[0,-0.1,0]]),UZ,0))     # 2
uLINK.append(LINK("RLEG_J1",1,0,4,np.transpose([[0,0,0]]),UX,0))        # 3
uLINK.append(LINK("RLEG_J2",5,0,5,np.transpose([[0,0,0]]),UY,0))        # 4
uLINK.append(LINK("RLEG_J3",1,0,6,np.transpose([[0,0,-0.3]]),UY,0))     # 5
uLINK.append(LINK("RLEG_J4",6,0,7,np.transpose([[0,0,-0.3]]),UY,0))     # 6
uLINK.append(LINK("RLEG_J5",2,0,0,np.transpose([[0,0,0]]),UX,0))        # 7

uLINK.append(LINK("LLEG_J0",5,0,9,np.transpose([[0,0.1,0]]),UZ,0))      # 8
uLINK.append(LINK("LLEG_J1",1,0,10,np.transpose([[0,0,0]]),UX,0))       # 9
uLINK.append(LINK("LLEG_J2",5,0,11,np.transpose([[0,0,0]]),UY,0))       # 10
uLINK.append(LINK("LLEG_J3",1,0,12,np.transpose([[0,0,-0.3]]),UY,0))    # 11
uLINK.append(LINK("LLEG_J4",6,0,13,np.transpose([[0,0,-0.3]]),UY,0))    # 12
uLINK.append(LINK("LLEG_J5",2,0,0,np.transpose([[0,0,0]]),UX,0))        # 13


# already checked, OK !
def Rodrigues(w,dt):
    norm_w = np.linalg.norm(w)  # norm vector
    eps = np.finfo(float).eps

    if norm_w < eps:
        R = np.identity(3)  # matriks identitas
        print(R)
    else:
        wn = w/norm_w       # rotation axis (unit vector)
        th = norm_w*dt      # amount of rotation (rad)
        w_wedge = [[0,-wn[2],wn[1]],
                [wn[2],0,-wn[0]],
                [-wn[1],wn[0],0]]
        R = np.identity(3)+np.dot(w_wedge,math.sin(th))+ np.dot(np.dot(w_wedge,w_wedge),(1-math.cos(th)))

    return R


# already checked, OK !
def FindMother(j):

    if j != 0:
        if j == 1:
            uLINK[j].mother = 0
            #print ("{}.mother = {}".format(uLINK[j].name,uLINK[j].mother))
        if uLINK[j].child != 0:
            uLINK[uLINK[j].child].mother = j
            #print ("{}.mother = {}".format(uLINK[uLINK[j].child].name,uLINK[uLINK[j].child].mother))
            FindMother(uLINK[j].child)
        if uLINK[j].sister != 0:
            uLINK[uLINK[j].sister].mother = uLINK[j].mother
            #print ("{}.mother = {}".format(uLINK[uLINK[j].sister].name,uLINK[uLINK[j].sister].mother))
            FindMother(uLINK[j].sister)

# =================================== ForwardKinematics(j) ==============================
# already checked, OK !
def ForwardKinematics(j):

    tanda = -1      # nilai bebas, yang penting harus lebih kecil dari 0 !

    if j == 0:
        tanda = 0
    elif j != 1:
        mom = uLINK[j].mother
        uLINK[j].p = np.dot(uLINK[mom].R,uLINK[j].b) + uLINK[mom].p
        uLINK[j].R = np.dot(uLINK[mom].R,Rodrigues(uLINK[j].a, uLINK[j].q))
        '''
        print("j = {}".format(j))
        print("posisi = ")
        print(uLINK[j].p)
        print("orientasi = ")
        print(uLINK[j].R)
        '''
        tanda = 1
    else:
        tanda = 1
    
    if tanda == 1:
        ForwardKinematics(uLINK[j].sister)
        ForwardKinematics(uLINK[j].child)
 
def FindRoute(to):
    idx_ = arr.array('i')
    i = uLINK[to].mother
    if i == 1:
        idx_.append(to)
    else:
        idx_ = FindRoute(i)
        idx_.append(to)

    return idx_

def CalcJacobian(idx):
    # Jacobian matrix of current configration in World frame
    jsize = len(idx)
    target = uLINK[idx[jsize-1]].p         # absolute target position
    J = np.zeros((6,jsize),dtype=float)
    
    for n in range(1,jsize+1):
        j = idx[n-1]
        a = np.dot(uLINK[j].R,uLINK[j].a)  # joint axis vector in world frame
        buff_ = np.transpose(np.cross(np.transpose(a),np.transpose(target - uLINK[j].p)))
        J[0:3,n-1] = buff_[:,0]
        J[3:6,n-1] = a[:,0]

    return J

def rot2omega(R):
    # T.Sugihara "Solvability-unconcerned Inverse Kinemacics based on 
    # Levenberg-Marquardt method with Robust Damping," Humanoids 2009
    el = [[R[2][1]-R[1][2]],        # index dikurangi 1, karena di python dimulai dari 0, sedangkan di matlab 1
        [R[0][2]-R[2][0]],
        [R[1][0]-R[0][1]]]
    norm_el = np.linalg.norm(el)
    eps = sys.float_info.epsilon

    if norm_el > eps:
        w = math.atan2(norm_el, R.trace()-1)/norm_el*np.array(el)
    elif R[0][0]>0 and R[1][1]>0 and R[2][2]>0:
        w = np.transpose([[0,0,0]])
    else:
        w = pi/2*np.array([[R[0][0]+1],[R[1][1]+1],[R[2][2]+1]])

    return w

def CalcVWerr(Cref, Cnow):
    err = np.zeros((6,1),dtype=float)
    perr = Cref.p - Cnow.p
    Rerr = np.dot(np.transpose(Cnow.R), Cref.R)
    werr = np.dot(Cnow.R, rot2omega(Rerr))
    err[0:3,0] = perr[:,0]
    err[3:6,0] = werr[:,0]

    return err

def MoveJoints(idx, dq):
    for n in range(0,len(idx)):
        j = idx[n]
        uLINK[j].q = uLINK[j].q + dq[n]

def InverseKinematicsAll(to, Target):
    global uLINK
    lamda = 0.9
    ForwardKinematics(1)
    idx = FindRoute(to)
    for n in range(0,10):
        J = CalcJacobian(idx)
        err = CalcVWerr(Target, uLINK[to])
        if (np.linalg.norm(err) < 1E-6):        #1E-6
            break
        J_inv = np.linalg.pinv(J) 
        buffJ_inv = J_inv.dot(err)
        dq = lamda * buffJ_inv
        MoveJoints(idx, dq)
        ForwardKinematics(1)

    #--- Relative velocity with respect to the body
    vd = Target.v - uLINK[1].v - np.transpose(np.cross(np.transpose(uLINK[1].w), np.transpose(Target.p - uLINK[1].p)))
    wd = Target.w - uLINK[1].w
    J  = CalcJacobian(idx)
    vdwd = np.zeros((6,1),dtype=float)
    vdwd[0:3,0] = vd[:,0]
    vdwd[3:6,0] = wd[:,0]
    J_inv = np.linalg.pinv(J)
    vq = J_inv.dot(vdwd)
    print(idx)
    for n in range(1,len(idx)+1):
        j = idx[n-1]
        uLINK[j].dq = vq[n-1]

def RPY2R(rpy):
    rot = np.zeros((3,3),dtype=float)
    # convert degree to radian
    roll  = rpy[0]*ToRad    
    pitch = rpy[1]*ToRad
    yaw   = rpy[2]*ToRad
    # rumus sin cos
    Cr = math.cos(roll)     
    Sr = math.sin(roll)
    Cp = math.cos(pitch) 
    Sp = math.sin(pitch)
    Cy = math.cos(yaw)   
    Sy = math.sin(yaw)
    # hitung matriks rotasi
    rot[0][0] = Cr*Cp       
    rot[0][1] = -Sr*Cy+Cr*Sp*Sy
    rot[0][2] = Sr*Sy+Cr*Sp*Cy
    rot[1][0] = Sr*Cp
    rot[1][1] = Cr*Cy+Sr*Sp*Sy
    rot[1][2] = -Cr*Sy+Sr*Sp*Cy
    rot[2][0] = -Sp
    rot[2][1] = Cp*Sy
    rot[2][2] = Cp*Cy
    
    return rot

def rot2RPY(r):
    rpy = np.zeros((3,1),dtype=float)
    roll  = math.atan2(r[1][0],r[0][0])*ToDeg
    pitch = math.atan2(-r[2][0],math.sqrt(r[2][1]**(2)+r[2][2]**(2)))*ToDeg
    yaw   = math.atan2(r[2][1],r[2][2])*ToDeg
    rpy[0] = roll
    rpy[1] = pitch
    rpy[2] = yaw
    
    return rpy

def set_b(L1,L2,L3,L4):
    global uLINK

    uLINK[1].p = np.transpose([[0,0,L2+L3+L4]])   # BODY
    uLINK[1].b = np.transpose([[0,0,L2+L3+L4]])
    
    uLINK[2].b = np.transpose([[0,-L1,0]])  # Right Leg
    uLINK[3].b = np.transpose([[0,0,0]])
    uLINK[4].b = np.transpose([[0,0,0]])
    uLINK[5].b = np.transpose([[0,0,-L2]])
    uLINK[6].b = np.transpose([[0,0,-L3]])
    uLINK[7].b = np.transpose([[0,0,0]])

    uLINK[8].b = np.transpose([[0,L1,0]])    # Left Leg
    uLINK[9].b = np.transpose([[0,0,0]])
    uLINK[10].b = np.transpose([[0,0,0]])
    uLINK[11].b = np.transpose([[0,0,-L2]])
    uLINK[12].b = np.transpose([[0,0,-L3]])
    uLINK[13].b = np.transpose([[0,0,0]])

def ToDeg2(servo_value):         # convert nilai servo digital langsung ke sudut
    deg = float(360)/1024        
    deg = (servo_value-512)*deg
    return deg

def ToRad2(servo_value):         # convert nilai servo digital langsung ke radian
    rad = float(2)*math.pi/1024  
    rad = (servo_value-512)*rad
    return rad
# ============ sampai sini =================================

def usleep(microseconds):
    #sleep for 1 microsecond
    time.sleep(microseconds * 10**(-6))

def msleep(milliseconds):
    #sleep for 1 millisecond
    time.sleep(milliseconds * 10**(-3))
    
def dxl_init(): # common AX atau MX, JIKA menggunakan protokol 1.0
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

def dxl_enableTorque(id): # common AX atau MX, JIKA menggunakan protokol 1.0
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    portHandler.setBaudRate(BAUDRATE)
    dxl_comm_result = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
    print("enable torque!")

def dxl_disableTorque(id): # common AX atau MX, JIKA menggunakan protokol 1.0
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    portHandler.setBaudRate(BAUDRATE)
    dxl_comm_result = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
    print("disable torque!")

def dxl_closePort(): # common AX atau MX, JIKA menggunakan protokol 1.0
    portHandler = PortHandler(DEVICENAME)
    portHandler.setBaudRate(BAUDRATE)
    portHandler.closePort()
    print("Close Port...")

def read_goalPos(dxl_type, id):
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    portHandler.setBaudRate(BAUDRATE)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, id, ADDR_AX_PRESENT_POSITION)

    if dxl_type=="AX":
        sudut = int(F(dxl_present_position,1024)*360)
        print("[ID:%03d] sudut:%03d PresPos:%03d" % (id, sudut, dxl_present_position))
    elif dxl_type=="MX":
        sudut = int(F(dxl_present_position,4096)*360)
        print("[ID:%03d] sudut:%03d PresPos:%03d" % (id, sudut, dxl_present_position))
    else:
        print("wrong input!")
        print("Press any key to terminate...")
        getch()

def write_goalPos(goalPosition):
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    # Allocate goal position value into byte array
    x = len(goalPosition)
    for i in range(0,x,3):
        # PENTING!
        # goalPosition[i]   = ID servo
        # goalPosition[i+1] = sudut servo       (range: 0~1023 atau 0~360 derajat)
        # goalPosition[i+2] = kecepatan servo   (range: 0~1023 atau 0~114 rpm)
        
        # Add Dynamixel goal position value (goalPos & speed!) to the Syncwrite parameter storage
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goalPosition[i+1])), DXL_HIBYTE(DXL_LOWORD(goalPosition[i+1])), DXL_LOBYTE(DXL_LOWORD(goalPosition[i+2])), DXL_HIBYTE(DXL_LOWORD(goalPosition[i+2]))]
        dxl_addparam_result = groupSyncWrite.addParam(goalPosition[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % x)
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def getValues(timeStamp):
    if timeStamp == 'true':
        ser2.write(b'c')                             # request arduino untuk kirim data!
        arduinoOutput = ser2.readline().split('-')   # misal: ['347a', '694b\r\n']
 
    return arduinoOutput

def getValuesK(timeStamp):
    if timeStamp == 'true':
        ser2.write(b'k')                             # request arduino untuk kirim data!
        arduinoOutput1 = ser2.readline().split('-')   # misal: ['347a', '694b\r\n']
 
    return arduinoOutput1

# ========================= Fungsi Inverse Kinematics ===============================
def Rodrigues(w,dt):
    norm_w = np.linalg.norm(w)  # norm vector
    eps = np.finfo(float).eps

    if norm_w < eps:
        R = np.identity(3)  # matriks identitas
        print(R)
    else:
        wn = w/norm_w       # rotation axis (unit vector)
        th = norm_w*dt      # amount of rotation (rad)
        w_wedge = [[0,-wn[2],wn[1]],
                [wn[2],0,-wn[0]],
                [-wn[1],wn[0],0]]
        R = np.identity(3)+np.dot(w_wedge,math.sin(th))+ np.dot(np.dot(w_wedge,w_wedge),(1-math.cos(th)))

    return R

def FindMother(j):

    if j != 0:
        if j == 1:
            uLINK[j].mother = 0
            #print ("{}.mother = {}".format(uLINK[j].name,uLINK[j].mother))
        if uLINK[j].child != 0:
            uLINK[uLINK[j].child].mother = j
            #print ("{}.mother = {}".format(uLINK[uLINK[j].child].name,uLINK[uLINK[j].child].mother))
            FindMother(uLINK[j].child)
        if uLINK[j].sister != 0:
            uLINK[uLINK[j].sister].mother = uLINK[j].mother
            #print ("{}.mother = {}".format(uLINK[uLINK[j].sister].name,uLINK[uLINK[j].sister].mother))
            FindMother(uLINK[j].sister)


# =================================== ForwardKinematics(j) ==============================
def ForwardKinematics(j):

    tanda = -1      # nilai bebas, yang penting harus lebih kecil dari 0 !

    if j == 0:
        tanda = 0
    elif j != 1:
        mom = uLINK[j].mother
        uLINK[j].p = np.dot(uLINK[mom].R,uLINK[j].b) + uLINK[mom].p
        uLINK[j].R = np.dot(uLINK[mom].R,Rodrigues(uLINK[j].a, uLINK[j].q))
        '''
        print("j = {}".format(j))
        print("posisi = ")
        print(uLINK[j].p)
        print("orientasi = ")
        print(uLINK[j].R)
        '''
        tanda = 1
    else:
        tanda = 1
    
    if tanda == 1:
        ForwardKinematics(uLINK[j].sister)
        ForwardKinematics(uLINK[j].child)
 
def FindRoute(to):
    idx_ = arr.array('i')
    i = uLINK[to].mother
    if i == 1:
        idx_.append(to)
    else:
        idx_ = FindRoute(i)
        idx_.append(to)

    return idx_

def CalcJacobian(idx):
    # Jacobian matrix of current configration in World frame
    jsize = len(idx)
    target = uLINK[idx[jsize-1]].p         # absolute target position
    J = np.zeros((6,jsize),dtype=float)
    
    for n in range(1,jsize+1):
        j = idx[n-1]
        a = np.dot(uLINK[j].R,uLINK[j].a)  # joint axis vector in world frame
        buff_ = np.transpose(np.cross(np.transpose(a),np.transpose(target - uLINK[j].p)))
        J[0:3,n-1] = buff_[:,0]
        J[3:6,n-1] = a[:,0]

    return J

def rot2omega(R):
    # ====================== rot2omega =============================
    #function w = rot2omega(R)
    # T.Sugihara "Solvability-unconcerned Inverse Kinemacics based on 
    # Levenberg-Marquardt method with Robust Damping," Humanoids 2009
    el = [[R[2][1]-R[1][2]],        # index dikurangi 1, karena di python dimulai dari 0, sedangkan di matlab 1
        [R[0][2]-R[2][0]],
        [R[1][0]-R[0][1]]]
    norm_el = np.linalg.norm(el)
    eps = sys.float_info.epsilon

    if norm_el > eps:
        w = math.atan2(norm_el, R.trace()-1)/norm_el*np.array(el)
    elif R[0][0]>0 and R[1][1]>0 and R[2][2]>0:
        w = np.transpose([[0,0,0]])
    else:
        w = pi/2*np.array([[R[0][0]+1],[R[1][1]+1],[R[2][2]+1]])

    return w

def CalcVWerr(Cref, Cnow):
    err = np.zeros((6,1),dtype=float)
    perr = Cref.p - Cnow.p
    Rerr = np.dot(np.transpose(Cnow.R), Cref.R)
    werr = np.dot(Cnow.R, rot2omega(Rerr))
    err[0:3,0] = perr[:,0]
    err[3:6,0] = werr[:,0]

    return err

def MoveJoints(idx, dq):
    for n in range(0,len(idx)):
        j = idx[n]
        uLINK[j].q = uLINK[j].q + dq[n]

def InverseKinematicsAll(to, Target):
    global uLINK
    lamda = 0.9
    ForwardKinematics(1)
    idx = FindRoute(to)
    #print("idx = ")
    #print(idx)
    for n in range(0,10):
        J = CalcJacobian(idx)
        err = CalcVWerr(Target, uLINK[to])
        if (np.linalg.norm(err) < 1E-6):        #1E-6
            break
        J_inv = np.linalg.pinv(J) 
        buffJ_inv = J_inv.dot(err)
        dq = lamda * buffJ_inv
        MoveJoints(idx, dq)
        ForwardKinematics(1)

    #--- Relative velocity with respect to the body
    vd = Target.v - uLINK[1].v - np.transpose(np.cross(np.transpose(uLINK[1].w), np.transpose(Target.p - uLINK[1].p)))
    wd = Target.w - uLINK[1].w
    J  = CalcJacobian(idx)
    vdwd = np.zeros((6,1),dtype=float)
    vdwd[0:3,0] = vd[:,0]
    vdwd[3:6,0] = wd[:,0]
    J_inv = np.linalg.pinv(J)
    vq = J_inv.dot(vdwd)
    #print("sudut IK = ")
    for n in range(1,len(idx)+1):
        j = idx[n-1]
        uLINK[j].dq = vq[n-1]
        #print(j)
        #print(uLINK[j].q)

def RPY2R(rpy):
    rot = np.zeros((3,3),dtype=float)

    # urutan rotasi : Z-Y-X
    # Rot(z,psi) -> step 1
    # Rot(y,the) -> step 2
    # Rot(x,phi) -> step 3
    # convert degree to radian
    phi   = rpy[0]*ToRad    # roll (rotasi terhadap sumbu x)    
    theta = rpy[1]*ToRad    # pitch (rotasi terhadap sumbu y)
    psi   = rpy[2]*ToRad    # yaw (rotasi terhadap sumbu z)

    # rumus sin cos
    Cphi = math.cos(phi)     
    Sphi = math.sin(phi)
    Cthe = math.cos(theta) 
    Sthe = math.sin(theta)
    Cpsi = math.cos(psi)   
    Spsi = math.sin(psi)

    # rumus sin cos ver2
    # hitung matriks rotasi (RUMUS YOUTUBE!)
    # Youtube: Christopher Lum
    # https://www.youtube.com/watch?v=GJBc6z6p0KQ
    rot[0][0] = Cthe*Cpsi
    rot[0][1] = Cthe*Spsi
    rot[0][2] = -Sthe
    rot[1][0] = Cpsi*Sthe*Sphi-Cphi*Spsi
    rot[1][1] = Cphi*Cpsi+Sthe*Sphi*Spsi
    rot[1][2] = Cthe*Sphi
    rot[2][0] = Cphi*Cpsi*Sthe+Sphi*Spsi
    rot[2][1] = -Cpsi*Sphi+Cphi*Sthe*Spsi
    rot[2][2] = Cthe*Cphi

    return rot

def rot2RPY(r):
    rpy = np.zeros((3,1),dtype=float)
    # ver2 
    # Youtube: Christopher Lum
    # https://www.youtube.com/watch?v=GJBc6z6p0KQ
    phi   = math.atan2(r[1][2],r[2][2])*ToDeg
    theta = -math.asin(r[0][2])*ToDeg
    psi   = math.atan2(r[0][1],r[0][0])*ToDeg
    rpy[0] = phi    # roll
    rpy[1] = theta  # pitch
    rpy[2] = psi    # yaw
    
    return rpy

def set_b(L1,L2,L3,L4):
    global uLINK

    uLINK[1].p = np.transpose([[0,0,L2+L3+L4]])   # BODY
    uLINK[1].b = np.transpose([[0,0,L2+L3+L4]])
    
    uLINK[2].b = np.transpose([[0,-L1,0]])  # Right Leg
    uLINK[3].b = np.transpose([[0,0,0]])
    uLINK[4].b = np.transpose([[0,0,0]])
    uLINK[5].b = np.transpose([[0,0,-L2]])
    uLINK[6].b = np.transpose([[0,0,-L3]])
    uLINK[7].b = np.transpose([[0,0,0]])

    uLINK[8].b = np.transpose([[0,L1,0]])    # Left Leg
    uLINK[9].b = np.transpose([[0,0,0]])
    uLINK[10].b = np.transpose([[0,0,0]])
    uLINK[11].b = np.transpose([[0,0,-L2]])
    uLINK[12].b = np.transpose([[0,0,-L3]])
    uLINK[13].b = np.transpose([[0,0,0]])

def ToDeg2(servo_value):         # convert nilai servo digital langsung ke sudut
    deg = float(360)/1024      
    deg = (servo_value-512)*deg
    return deg

def ToRad2(servo_value):         # convert nilai servo digital langsung ke radian
    rad = float(2)*math.pi/1024  
    rad = (servo_value-512)*rad
    #print(rad)
    return rad

def ToRad3(servo_value,sp):         # convert nilai servo digital langsung ke radian
    rad = float(2)*math.pi/1024  
    rad = (servo_value-sp)*rad
    #print(rad)
    return rad

def ToRad4(servo_value):         # convert nilai servo digital langsung ke radian
    rad = float(2)*math.pi/1024  
    rad = (-servo_value-512)*rad
    #print(rad)
    return rad

def ToDeg3(rad):                    # convert radian langsung ke nilai servo digital
    servo_value = rad/(float(2)*math.pi/1024)+512
    #print(servo_value)
    return servo_value

def ToDeg4(rad,sp):                    # convert radian langsung ke nilai servo digital
    servo_value = rad/(float(2)*math.pi/1024)+sp
    #print(servo_value)
    return servo_value

def ToDeg5(rad):                    # convert radian langsung ke nilai servo digital
    servo_value = rad/(float(2)*math.pi/1024)+512
    servo_value = -servo_value
    #print(servo_value)
    return servo_value

def radToDegAll(to):
    global uLINK
    idx = FindRoute(to)
    #print("idx = ")
    #print(idx)
    #print("print sudut manual")
    for n in range(1,len(idx)+1):
        j = idx[n-1]
        #print(j)
        if j == 2:
            uLINK[j].q = ToDeg4(uLINK[j].q,spka)
        elif j == 8:
            uLINK[j].q = ToDeg4(uLINK[j].q,spki)
        elif j == 5 or j == 10 or j ==  6:
            uLINK[j].q = ToDeg5(uLINK[j].q)
        else:
            uLINK[j].q = ToDeg3(uLINK[j].q)


def degToRadAll(to,arrServo):
    idx = FindRoute(to)
    #print("idx = ")
    #print(idx)
    #print("print sudut manual")
    for n in range(1,len(idx)+1):
        j = idx[n-1]
        #print(j)
        if j == 2:
            uLINK[j].q = ToRad3(arrServo[n-1],spka)
        elif j == 8:
            uLINK[j].q = ToRad3(arrServo[n-1],spki)
        elif j == 5 or j == 10 or j == 6:
            uLINK[j].q = ToRad4(arrServo[n-1])
        else:
            uLINK[j].q = ToRad2(arrServo[n-1])

def toMX(sudutAX):
    sudutMX = int(sudutAX*4095/1023)
    return sudutMX

def toAX(sudutMX):
    sudutAX = int(sudutMX*1024/4095)
    return sudutAX

def toMXspeed(speedAX):
    speedMX = int(speedAX*116.62/114)
    return speedMX

def sorting_servo(goalPoss):
    L = []                                         # buffer list untuk sorting ID servo
    x = len(goalPoss)
    for i in range(0,x,3):
        if(goalPoss[i]>=7 and goalPoss[i]<=18):     # servo yang akan dihitung IK nya (12 servo)
            L.append((goalPoss[i],goalPoss[i+1]))
            L.sort(key=lambda x: x[0])
    # sorting selesai, selanjutnya salin data list ke variabel baru (arrServoKa dan arrServoKi)
    arrServoKa = np.zeros((6,1),dtype=float)
    arrServoKi = np.zeros((6,1),dtype=float)
    idx1 = 0
    idx2 = 0
    for i in range(0,len(L),1):
        if i % 2 == 0:
            if idx1>=2 and idx1<=4:
                arrServoKa[idx1] = toAX(L[i][1])
            else:
                arrServoKa[idx1] = L[i][1]
            idx1=idx1+1
        else:
            if idx2>=2 and idx2<=4:
                arrServoKi[idx2] = toAX(L[i][1])
            else:
                arrServoKi[idx2] = L[i][1]
            idx2=idx2+1

    return arrServoKa,arrServoKi

def sorting_servo2(goalPoss,alfa,k):
    # SORTING, kemudian ADJUST nilai servo tangan dan paha sebesar alfa
    # ============== TEST SORTING_SERVO2(arrServoKa,arrServoKi) =============
    idxka = FindRoute(7)        # indeks kaki kanan
    idxki = FindRoute(13)       # indeks kaki kiri
    L = []                      # buffer list untuk sorting ID servo
    goalPossUpdate = []
    x = len(goalPoss)
    for i in range(0,x,3):
        if(goalPoss[i]>=1 and goalPoss[i]<=2):     # servo yang akan dihitung IK nya (12 servo)
            L.append((goalPoss[i],goalPoss[i+1],goalPoss[i+2]))
            L.sort(key=lambda x: x[0])
        elif(goalPoss[i]>=7 and goalPoss[i]<=18):     # servo yang akan dihitung IK nya (12 servo)
            L.append((goalPoss[i],goalPoss[i+1],goalPoss[i+2]))
            L.sort(key=lambda x: x[0])
    y = len(L)
    idx1 = 0
    idx2 = 0
    j = 0
    for i in range(0,y):
        goalPossUpdate.append(L[i][0])
        if i >= 2:
            if i % 2 == 0:
                j = idxka[idx1]
                if idx1>=2 and idx1<=4:
                    goalPossUpdate.append(toMX(int(uLINK[j].q)))
                else:
                    goalPossUpdate.append(int(uLINK[j].q))
                idx1=idx1+1
            else:
                j = idxki[idx2]
                if idx2>=2 and idx2<=4:
                    goalPossUpdate.append(toMX(int(uLINK[j].q)))
                else:
                    goalPossUpdate.append(int(uLINK[j].q))
                idx2=idx2+1
        else:
            goalPossUpdate.append(L[i][1])

        goalPossUpdate.append(L[i][2])

    # ADJUST nilai servo tangan dan paha sebesar alfa
    # ID1-alfa*k
    # ID2+alfa*k
    # ID11+toMX(alfa)
    # ID12-toMX(alfa)
    goalPossUpdate[1] = goalPossUpdate[1]-alfa*k
    goalPossUpdate[4] = goalPossUpdate[4]+alfa*k
    goalPossUpdate[19] = goalPossUpdate[19]+toMX(alfa)      # cek tanda +- di bagian ini [recheck sudah OK !]
    goalPossUpdate[22] = goalPossUpdate[22]-toMX(alfa)      # cek tanda +- di bagian ini [recheck sudah OK !]

    return goalPossUpdate

def cek_maxservo(goalPossNew):
    maxServo = [1,0,1023,2,0,1023,7,225,570,8,454,799,9,364,660,10,364,660,11,976,2320,12,1776,3120,13,646,2048,14,2048,3450,15,1833,3164,16,932,2263,17,383,665,18,383,665]
    # ================= CEK maksimum sudut untuk tiap servo =======================
    # agar robot tidak rusak karena sudutnya offside !
    x = len(maxServo)
    for i in range(0,x,3):
        if (goalPossNew[i+1] > maxServo[i+2]):
            goalPossNew[i+1] = maxServo[i+2]
        if (goalPossNew[i+1] < maxServo[i+1]):
            goalPossNew[i+1] = maxServo[i+1]

    return goalPossNew

# =================================== Fuzzy Logic Controler =============================
def kontrol_fuzzy(kontrol):
    global mulai_baca
    global baca_sensor
    global goalPoss
    global full_body
    global uLINK
    #global ver2
    global verMX
    global L1
    global L2
    global L3
    global L4
    global spka
    global spki
    global walkready
    global fml2_6_
    global fmr2_6_
    cek_sensor = 'false'
    kalibrasi = 'false'
    timeStamp = 'true'

    if kontrol == 1:
        cek_sensor = 'true'
        #print("============MODE CEK SENSOR====================")
    elif kontrol == 2:
        kalibrasi = 'true'
        print("============MODE KALIBRASI=====================")
    else:
        print("============MODE KONTROL TIDAK AKTIF============")

    # ==================================== MODE Kalibrasi ====================================
    if kalibrasi == 'true':
        #print("============STEP 1")
        print("proses kalibrasi ....")

        if baca_sensor == 0:
            #print("============STEP 2")
            data = getValues(timeStamp)
            print(data)
            #print("AKU SALAH MASUK!")
            data_init = data[0]                     # 'READY' atau '13a' atau data karakter lainnya
            data_init = data_init.strip()           # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms
            val_init1 = data_init[:-1]              # nunggu sampai ada 'READY'
            val_init2 = data_init[-1:]              # nunggu sampai ada 'a'

            while val_init1 != 'READY' and val_init2 != 'a':
                #print("============STEP 3")
                data = getValues(timeStamp)
                print(data)
                data_init = data[0]                     # 'READY' atau '13a' atau data karakter lainnya
                data_init = data_init.strip()           # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms
                val_init1 = data_init[:-1]              # nunggu sampai ada 'READY'
                val_init2 = data_init[-1:]              # nunggu sampai ada 'a'
                #print("NONGKRONG di while nich!")

            if val_init1 == 'READY' or val_init2 == 'a':
                #print("============STEP 4")
                mulai_baca=1
                baca_sensor =1
                # Kalo mode cek sensor dipanggil di awal, maka program akan masuk ke bawah sini dulu, baru ke bawah [STEP 1 -> STEP 2 -> STEP 3 -> STEP 4 -> STEP 5]
            else:
                mulai_baca=0
        if baca_sensor == 1:
            # Kalo mode cek sensor tidak dipanggil di awal, maka program langsung masuk ke bawah sini [STEP 1 -> STEP 5]
            #print("============STEP 5")
            data = getValuesK(timeStamp)
            print(data)

            # ============== Dari sini ============================
            data_a = data[0]                # 347a
            data_b = data[1]                # 694b
            data_c = data[2]                # 694b
            data_d = data[3]                # 694b

            data_a = data_a.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms
            var_a = data_a[-1:]             # a
            val_a = data_a[:-1]             # 347
            #print(var_a)

            while(var_a!='p'):
                data = ser2.readline().split('-')   # misal: ['347a', '694b\r\n']
                print(data)

                data_a = data[0]                # 347a
                data_b = data[1]                # 694b
                data_c = data[2]                # 694b
                data_d = data[3]                # 694b
                    
                data_a = data_a.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms
                var_a = data_a[-1:]             # a
                val_a = data_a[:-1]             # 347
                #print(var_a)

            # ==================================
        kalibrasi = 'false'


    # ==================================== MODE Cek Sensor (Baca Sensor) =====================
    if cek_sensor == 'true':
        #print("============STEP 1")
        #print("proses baca sensor ....")

        if baca_sensor == 0:
            #print("============STEP 2")
            data = getValues(timeStamp)
            print(data)
            #print("AKU SALAH MASUK!")
            data_init = data[0]                     # 'READY' atau '13a' atau data karakter lainnya
            data_init = data_init.strip()           # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms
            val_init1 = data_init[:-1]              # nunggu sampai ada 'READY'
            val_init2 = data_init[-1:]              # nunggu sampai ada 'a'

            while val_init1 != 'READY' and val_init2 != 'a':
                #print("============STEP 3")
                data = getValues(timeStamp)
                print(data)
                data_init = data[0]                     # 'READY' atau '13a' atau data karakter lainnya
                data_init = data_init.strip()           # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms
                val_init1 = data_init[:-1]              # nunggu sampai ada 'READY'
                val_init2 = data_init[-1:]              # nunggu sampai ada 'a'
                #print("NONGKRONG di while nich!")

            if val_init1 == 'READY' or val_init2 == 'a':
                #print("============STEP 4")
                mulai_baca=1
                baca_sensor =1
                # Kalo mode cek sensor dipanggil di awal, maka program akan masuk ke bawah sini dulu, baru ke bawah [STEP 1 -> STEP 2 -> STEP 3 -> STEP 4 -> STEP 5]
            else:
                mulai_baca=0

        if baca_sensor == 1:
            # Kalo mode cek sensor tidak dipanggil di awal, maka program langsung masuk ke bawah sini [STEP 1 -> STEP 5]
            #print("============STEP 5")
            data = getValues(timeStamp)
            #print(data)

            data_a = data[0]                # 347a         # ============================== A
            data_b = data[1]                # 694b         # ============================== B
            data_c = data[2]                # 694b         # ============================== C
            data_d = data[3]                # 694b         # ============================== D
            data_e = data[4]                # 694b         # ============================== E

            data_a = data_a.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms   # ============================== A
            var_a = data_a[-1:]             # a
            val_a = data_a[:-1]             # 347
            
            data_b = data_b.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms   # ============================== B
            var_b = data_b[-1:]             # b
            val_b = data_b[:-1]             # 694
            
            data_c = data_c.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms   # ============================== C
            var_c = data_c[-2:]             # b
            val_c = data_c[:-2]             # 694

            data_d = data_d.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms   # ============================== D
            var_d = data_d[-1:]             # b
            val_d = data_d[:-1]             # 694
            
            data_e = data_e.strip() # we always separate the last two chars out! ex : 1021ms, 812ms, 12ms   # ============================== E
            var_e = data_e[-1:]             # b
            val_e = data_e[:-1]             # 694

            # *x,20,y,40#
            if var_a == 'a':                # if you use millis() in Arduino code convert to seconds        # ============================== A
                alfa_ = float(val_a)        # 347.0
            
            if var_b == 'b':                # if you use millis() in Arduino code convert to seconds        # ============================== B
                beta_ = float(val_b)        # 694.0
            
            if var_c == 'ms':               # if you use millisecond() in Arduino code convert to seconds   # ============================== C
                timeValue = float(val_c)
                timeValue = timeValue/1000
                #frek = 1/timeValue
            
            if var_d == 'd':                # if you use millis() in Arduino code convert to seconds        # ============================== D
                tanda_mpu = float(val_d)    # 694.0
                if tanda_mpu == 1:
                    tanda_mpu_x = 1
                else:
                    tanda_mpu_x = -1
            
            if var_e == 'e':                # if you use millis() in Arduino code convert to seconds     # ============================== A
                out_mpu = float(val_e)      # 694.0

            alfa = tanda_mpu_x*alfa_        
            beta = tanda_mpu_x*beta_        

            print(data)
            print(alfa)
            print(beta)
            print(out_mpu)
            
            # ============================ Inverse Kinematics, Forward Kinematics ====================
            # nilai alfa, beta, k hasil perhitungan fuzzy dari arduino !
            alfa = int(alfa)     #alfa*20
            beta = float(beta)
            k = 20
            walkready = 1
            # ================= 0. Sorting ID sebelum proses hitung FK dan IK
            # input goalPoss
            if walkready ==  1:
                goalPoss = [1,235,72,2,788,72,3,279,72,4,744,72,5,462,72,6,561,72,7,358,72,8,666,72,9,507,72,17,507,72,10,516,72,18,516,72,11,toMX(346),toMXspeed(72),12,toMX(677),toMXspeed(72),13,toMX(240),toMXspeed(220),14,toMX(783),toMXspeed(220),15,toMX(647),toMXspeed(120),16,toMX(376),toMXspeed(120)]
                walkready = 1
            elif fml2_6_ == 1:
                goalPoss = [1,294,speed,2,847,speed,7,358,speed,8,666,speed,9,507,speed,10,516,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(248),toMXspeed(speed),14,toMX(775),toMXspeed(speed),15,toMX(671),toMXspeed(speed),16,toMX(409),toMXspeed(speed),17,507,speed,18,516,speed]
                fml2_6_ = 0
            elif fmr2_6_ == 1:
                goalPoss = [1,175,speed,2,728,speed,7,358,speed,8,666,speed,9,507,speed,10,516,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(248),toMXspeed(speed),14,toMX(775),toMXspeed(speed),15,toMX(614),toMXspeed(speed),16,toMX(352),toMXspeed(speed),17,507,speed,18,516,speed]
                fmr2_6_ = 0
            #print(goalPoss)

            # sorting ID, pilih 12 servo kaki untuk hitung IK (inverse kinematics)
            arrServoKa,arrServoKi =  sorting_servo(goalPoss)
            degToRadAll(7,arrServoKa)   # ============================== A
            degToRadAll(13,arrServoKi)  # ============================== B

            # ================================= Forward Kinematics ======================================
            FindMother(1)
            ForwardKinematics(1)

            # ================================= Inverse Kinematics ======================================
            # inisialisasi untuk Inverse Kinematics
            uLINK[1].v = np.transpose([[0, 0, 0]])              # BODY
            uLINK[1].w = np.transpose([[0, 0, 0]])              # BODY
            Rfoot = foot()                      # ============================== A
            Lfoot = foot()                      # ============================== B

            # input Kaki Kanan IK
            # adjust posisi x dari kaki Kanan sebesar beta
            #j = 7                               # ============================== A
            Rfoot.p = uLINK[7].p 
            Rfoot.p[0] = Rfoot.p[0]-beta        # + offset_x_hip (posisi koordinat) [tanda beta, kebalikan sama yang punya fahmizal ! INGAT !]
            Rfoot.R = uLINK[7].R
            #print("Rfoot.R = ")
            #print(Rfoot.R)
            Rfoot.v = np.transpose([[0,0,0]])
            Rfoot.w = np.transpose([[0,0,0]])

            # input Kaki Kiri IK
            # adjust posisi x dari kaki Kiri sebesar beta
            #j = 13                              # ============================== B
            Lfoot.p = uLINK[13].p 
            Lfoot.p[0] = Lfoot.p[0]-beta        # + offset_x_hip (posisi koordinat) [tanda beta, kebalikan sama yang punya fahmizal ! INGAT !]
            Lfoot.R = uLINK[13].R 
            #print("Lfoot.R = ")
            #print(Lfoot.R)
            Lfoot.v = np.transpose([[0,0,0]])
            Lfoot.w = np.transpose([[0,0,0]])

            #print("============== Result Inverse Kinematics ============")
            InverseKinematicsAll(7, Rfoot)      # 7 = RLEG_J5      # ============================== A
            InverseKinematicsAll(13, Lfoot)     # 13 = LLEG_J5     # ============================== B
            radToDegAll(7)                      # convert radian ke nilai servo digital untuk kaki KANAN       # ============================== A
            radToDegAll(13)                     # convert radian ke nilai servo digital untuk kaki KIRI        # ============================== B

            # kirim sudut ke SERVO ! 
            #print("============== Kirim sudut ke servo AX MX ! =========")
            goalPossNew = sorting_servo2(goalPoss,alfa,k)   # adjust sudut tangan dan paha sebesar alfa
            goalPossNew = cek_maxservo(goalPossNew)         # cek maksimum sudut AX dan MX yang diperbolehkan, agar tidak merusak robot!
            #print(goalPossNew)
            write_goalPos(goalPossNew)
            
            
            # ============================ kirim sudut ke servo AX dan MX ============================

            # =============== gerakkan servo AX12 ==========================
            
        cek_sensor = 'false'


#------------------ void FML1_0() --------------------------
def FML1_0():
    global full_body
    global y
    global speed
    global verMX
    
    ID11 = 62 + y
    ID12 = 132- y
    
    if full_body == 1:
        if verMX == 1:  #versi MX mode default
            goalPoss = [1,169,speed,2,722,speed,3,279,speed,4,744,speed,5,462,speed,6,561,speed,7,358,speed,8,666,speed,9,513,speed,17,201,speed,10,522,speed,18,522,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(246),toMXspeed(speed),14,toMX(769),toMXspeed(speed),15,toMX(621),toMXspeed(speed),16,toMX(348),toMXspeed(speed)]
        else :          #versi AX mode default
            goalPoss = [1,169,speed,2,722,speed,3,279,speed,4,744,speed,5,462,speed,6,561,speed,7,358,speed,8,666,speed,9,513,speed,17,201,speed,10,522,speed,18,522,speed,11,256+ID11,speed,12,512+ID12,speed,13,246,speed,14,769,speed,15,621,speed,16,348,speed]
    else :
        goalPoss = [7,358,speed,8,666,speed,9,513,speed,17,201,speed,10,522,speed,18,522,speed,11,256+ID11,speed,12,512+ID12,speed,13,246,speed,14,769,speed,15,621,speed,16,348,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML1_1() --------------------------
def FML1_1():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 68 + y
    ID12 = 123- y
    
    if full_body == 1:
        if verMX == 1:
            goalPoss = [1,166,speed,2,719,speed,9,518,speed,10,527,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(248),toMXspeed(speed),14,toMX(761),toMXspeed(speed),15,toMX(625),toMXspeed(speed),16,toMX(348),toMXspeed(speed),17,518,speed,18,527,speed]
        else:
            goalPoss = [1,166,speed,2,719,speed,9,518,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,761,speed,15,625,speed,16,348,speed,17,518,speed,18,527,speed]
    else :
        goalPoss = [9,518,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,761,speed,15,625,speed,16,348,speed,17,518,speed,18,527,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML1_2() --------------------------
def FML1_2():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 74 + y
    ID12 = 122- y
    
    if full_body == 1:
        if verMX == 1:
            goalPoss = [1,167,speed,2,720,speed,9,519,speed,10,533,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(252),toMXspeed(speed),14,toMX(761),toMXspeed(speed),15,toMX(628),toMXspeed(speed),16,toMX(346),toMXspeed(speed),17,523,speed,18,532,speed]
        else:
            goalPoss = [1,167,speed,2,720,speed,9,519,speed,10,533,speed,11,256+ID11,speed,12,512+ID12,speed,13,252,speed,14,761,speed,15,628,speed,16,346,speed,17,523,speed,18,532,speed]
    else :
        goalPoss = [9,519,speed,10,533,speed,11,256+ID11,speed,12,512+ID12,speed,13,252,speed,14,761,speed,15,628,speed,16,346,speed,17,523,speed,18,532,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML1_3() --------------------------
def FML1_3():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 81 + y
    ID12 = 134- y
    
    if full_body == 1:
        if verMX == 1:
            goalPoss = [1,171,speed,2,724,speed,9,513,speed,10,541,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(257),toMXspeed(speed),14,toMX(778),toMXspeed(speed),15,toMX(629),toMXspeed(speed),16,toMX(341),toMXspeed(speed),17,527,speed,18,537,speed]
        else:
            goalPoss = [1,171,speed,2,724,speed,9,513,speed,10,541,speed,11,256+ID11,speed,12,512+ID12,speed,13,257,speed,14,778,speed,15,629,speed,16,341,speed,17,527,speed,18,537,speed]
    else :
        goalPoss = [9,513,speed,10,541,speed,11,256+ID11,speed,12,512+ID12,speed,13,257,speed,14,778,speed,15,629,speed,16,341,speed,17,527,speed,18,537,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML1_4() --------------------------
def FML1_4():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 81 + y
    ID12 = 154- y
    
    if full_body == 1:
        if verMX == 1:
            goalPoss = [1,179,speed,2,732,speed,9,504,speed,10,550,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(262),toMXspeed(speed),14,toMX(801),toMXspeed(speed),15,toMX(630),toMXspeed(speed),16,toMX(338),toMXspeed(speed),17,530,speed,18,541,speed]
        else:
            goalPoss = [1,179,speed,2,732,speed,9,504,speed,10,550,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,801,speed,15,630,speed,16,338,speed,17,530,speed,18,541,speed]
    else :
        goalPoss = [9,504,speed,10,550,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,801,speed,15,630,speed,16,338,speed,17,530,speed,18,541,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML1_5() --------------------------
def FML1_5():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 92 + y
    ID12 = 176- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,189,speed,2,742,speed,9,496,speed,10,556,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(266),toMXspeed(speed),14,toMX(820),toMXspeed(speed),15,toMX(631),toMXspeed(speed),16,toMX(342),toMXspeed(speed),17,532,speed,18,543,speed]
        else:
            goalPoss = [1,189,speed,2,742,speed,9,496,speed,10,556,speed,11,256+ID11,speed,12,512+ID12,speed,13,266,speed,14,820,speed,15,631,speed,16,342,speed,17,532,speed,18,543,speed]
    else :
        goalPoss = [9,496,speed,10,556,speed,11,256+ID11,speed,12,512+ID12,speed,13,266,speed,14,820,speed,15,631,speed,16,342,speed,17,532,speed,18,543,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML1_6() --------------------------
def FML1_6():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 95 + y
    ID12 = 195- y
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,201,speed,2,754,speed,9,493,speed,10,558,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(267),toMXspeed(speed),14,toMX(827),toMXspeed(speed),15,toMX(633),toMXspeed(speed),16,toMX(354),toMXspeed(speed),17,533,speed,18,544,speed]
        else:
            goalPoss = [1,201,speed,2,754,speed,9,493,speed,10,558,speed,11,256+ID11,speed,12,512+ID12,speed,13,267,speed,14,827,speed,15,633,speed,16,354,speed,17,533,speed,18,544,speed]
    else :
        goalPoss = [9,493,speed,10,558,speed,11,256+ID11,speed,12,512+ID12,speed,13,267,speed,14,827,speed,15,633,speed,16,354,speed,17,533,speed,18,544,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_0() --------------------------
def FML2_0():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 97 + y
    ID12 = 208- y  
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,215,speed,2,768,speed,9,496,speed,10,556,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(266),toMXspeed(speed),14,toMX(820),toMXspeed(speed),15,toMX(636),toMXspeed(speed),16,toMX(374),toMXspeed(speed),17,532,speed,18,543,speed]
        else:
            goalPoss = [1,215,speed,2,768,speed,9,496,speed,10,556,speed,11,256+ID11,speed,12,512+ID12,speed,13,266,speed,14,820,speed,15,636,speed,16,374,speed,17,532,speed,18,543,speed]
    else :
        goalPoss = [9,496,speed,10,556,speed,11,256+ID11,speed,12,512+ID12,speed,13,266,speed,14,820,speed,15,636,speed,16,374,speed,17,532,speed,18,543,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_1() --------------------------
def FML2_1():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 98 + y
    ID12 = 211- y
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,230,speed,2,783,speed,9,504,speed,10,550,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(262),toMXspeed(speed),14,toMX(801),toMXspeed(speed),15,toMX(641),toMXspeed(speed),16,toMX(396),toMXspeed(speed),17,530,speed,18,541,speed]
        else:
            goalPoss = [1,230,speed,2,783,speed,9,504,speed,10,550,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,801,speed,15,641,speed,16,396,speed,17,530,speed,18,541,speed]
    else:
        goalPoss = [9,504,speed,10,550,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,801,speed,15,641,speed,16,396,speed,17,530,speed,18,541,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_2() --------------------------
def FML2_2():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 99 + y
    ID12 = 208- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,246,speed,2,799,speed,9,513,speed,10,541,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(257),toMXspeed(speed),14,toMX(778),toMXspeed(speed),15,toMX(647),toMXspeed(speed),16,toMX(416),toMXspeed(speed),17,527,speed,18,537,speed]
        else:
            goalPoss = [1,246,speed,2,799,speed,9,513,speed,10,541,speed,11,256+ID11,speed,12,512+ID12,speed,13,257,speed,14,778,speed,15,647,speed,16,416,speed,17,527,speed,18,537,speed]
    else :
        goalPoss = [9,513,speed,10,541,speed,11,256+ID11,speed,12,512+ID12,speed,13,257,speed,14,778,speed,15,647,speed,16,416,speed,17,527,speed,18,537,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_3() --------------------------
def FML2_3():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 100 + y
    ID12 = 204 - y
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,260,speed,2,573,speed,9,519,speed,10,533,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(252),toMXspeed(speed),14,toMX(761),toMXspeed(speed),15,toMX(654),toMXspeed(speed),16,toMX(428),toMXspeed(speed),17,523,speed,18,532,speed]
        else:
            goalPoss = [1,260,speed,2,573,speed,9,519,speed,10,533,speed,11,256+ID11,speed,12,512+ID12,speed,13,252,speed,14,761,speed,15,654,speed,16,428,speed,17,523,speed,18,532,speed]
    else:
        goalPoss = [9,519,speed,10,533,speed,11,256+ID11,speed,12,512+ID12,speed,13,252,speed,14,761,speed,15,654,speed,16,428,speed,17,523,speed,18,532,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_4() --------------------------
def FML2_4():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 103 + y
    ID12 = 202 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,274,speed,2,827,speed,9,518,speed,10,527,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(248),toMXspeed(speed),14,toMX(761),toMXspeed(speed),15,toMX(660),toMXspeed(speed),16,toMX(427),toMXspeed(speed),17,518,speed,18,527,speed]
        else:
            goalPoss = [1,274,speed,2,827,speed,9,518,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,761,speed,15,660,speed,16,427,speed,17,518,speed,18,527,speed]
    else :
        goalPoss = [9,518,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,761,speed,15,660,speed,16,427,speed,17,518,speed,18,527,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_5() --------------------------
def FML2_5():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 107 + y
    ID12 = 202 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,285,speed,2,838,speed,9,513,speed,10,522,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(246),toMXspeed(speed),14,toMX(769),toMXspeed(speed),15,toMX(666),toMXspeed(speed),16,toMX(418),toMXspeed(speed),17,513,speed,18,522,speed]
        else:
            goalPoss = [1,285,speed,2,838,speed,9,513,speed,10,522,speed,11,256+ID11,speed,12,512+ID12,speed,13,246,speed,14,769,speed,15,666,speed,16,418,speed,17,513,speed,18,522,speed]
    else:
        goalPoss = [9,513,speed,10,522,speed,11,256+ID11,speed,12,512+ID12,speed,13,246,speed,14,769,speed,15,666,speed,16,418,speed,17,513,speed,18,522,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FML2_6() --------------------------
def FML2_6():
    global full_body
    global y
    global speed
    global verMX
    global fml2_6_
    
    ID11 = 114 + y
    ID12 = 204 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,294,speed,2,847,speed,9,507,speed,17,507,speed,10,516,speed,18,516,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(248),toMXspeed(speed),14,toMX(775),toMXspeed(speed),15,toMX(671),toMXspeed(speed),16,toMX(409),toMXspeed(speed)]
        else:
            goalPoss = [1,294,speed,2,847,speed,9,507,speed,17,507,speed,10,516,speed,18,516,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,775,speed,15,671,speed,16,409,speed]
    else :
        goalPoss = [9,507,speed,17,507,speed,10,516,speed,18,516,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,775,speed,15,671,speed,16,409,speed]
    write_goalPos(goalPoss)
    msleep(37)
    fml2_6_ = 1

#------------------ void FMR1_0() --------------------------
def FMR1_0():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 123 + y
    ID12 = 193 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,300,speed,2,853,speed,9,501,speed,10,510,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(254),toMXspeed(speed),14,toMX(777),toMXspeed(speed),15,toMX(675),toMXspeed(speed),16,toMX(402),toMXspeed(speed),17,501,speed,18,510,speed]
        else:
            goalPoss = [1,300,speed,2,853,speed,9,501,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,254,speed,14,777,speed,15,675,speed,16,402,speed,17,501,speed,18,510,speed]
    else :
        goalPoss = [9,501,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,254,speed,14,777,speed,15,675,speed,16,402,speed,17,501,speed,18,510,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR1_1() --------------------------
def FMR1_1():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 132 + y
    ID12 = 187 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,303,speed,2,856,speed,9,496,speed,10,505,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(262),toMXspeed(speed),14,toMX(775),toMXspeed(speed),15,toMX(675),toMXspeed(speed),16,toMX(398),toMXspeed(speed),17,496,speed,18,505,speed]
        else:
            goalPoss = [1,303,speed,2,856,speed,9,496,speed,10,505,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,775,speed,15,675,speed,16,398,speed,17,496,speed,18,505,speed]
    else :
        goalPoss = [9,496,speed,10,505,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,775,speed,15,675,speed,16,398,speed,17,496,speed,18,505,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR1_2() --------------------------
def FMR1_2():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 133 + y
    ID12 = 181 - y
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,302,speed,2,855,speed,9,490,speed,10,504,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(262),toMXspeed(speed),14,toMX(771),toMXspeed(speed),15,toMX(677),toMXspeed(speed),16,toMX(395),toMXspeed(speed),17,491,speed,18,500,speed]
        else:
            goalPoss = [1,302,speed,2,855,speed,9,490,speed,10,504,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,771,speed,15,677,speed,16,395,speed,17,491,speed,18,500,speed]
    else:
        goalPoss = [9,490,speed,10,504,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,771,speed,15,677,speed,16,395,speed,17,491,speed,18,500,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR1_3() --------------------------
def FMR1_3():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 105 + y
    ID12 = 174 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,298,speed,2,851,speed,9,482,speed,10,510,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(245),toMXspeed(speed),14,toMX(766),toMXspeed(speed),15,toMX(682),toMXspeed(speed),16,toMX(394),toMXspeed(speed),17,486,speed,18,496,speed]
        else:
            goalPoss = [1,298,speed,2,851,speed,9,482,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,245,speed,14,766,speed,15,682,speed,16,394,speed,17,486,speed,18,496,speed]
    else :
        goalPoss = [9,482,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,245,speed,14,766,speed,15,682,speed,16,394,speed,17,486,speed,18,496,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR1_4() --------------------------
def FMR1_4():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 101 + y
    ID12 = 168 - y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,290,speed,2,843,speed,9,473,speed,10,519,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(222),toMXspeed(speed),14,toMX(761),toMXspeed(speed),15,toMX(685),toMXspeed(speed),16,toMX(393),toMXspeed(speed),17,482,speed,18,493,speed]
        else:
            goalPoss = [1,290,speed,2,843,speed,9,473,speed,10,519,speed,11,256+ID11,speed,12,512+ID12,speed,13,222,speed,14,761,speed,15,685,speed,16,393,speed,17,482,speed,18,493,speed]
    else :
        goalPoss = [9,473,speed,10,519,speed,11,256+ID11,speed,12,512+ID12,speed,13,222,speed,14,761,speed,15,685,speed,16,393,speed,17,482,speed,18,493,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR1_5() --------------------------
def FMR1_5():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 79 + y
    ID12 = 163- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,280,speed,2,833,speed,9,467,speed,10,527,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(203),toMXspeed(speed),14,toMX(757),toMXspeed(speed),15,toMX(681),toMXspeed(speed),16,toMX(392),toMXspeed(speed),17,480,speed,18,491,speed]
        else:
            goalPoss = [1,280,speed,2,833,speed,9,467,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,203,speed,14,757,speed,15,681,speed,16,392,speed,17,480,speed,18,491,speed]
    else :
        goalPoss = [9,467,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,203,speed,14,757,speed,15,681,speed,16,392,speed,17,480,speed,18,491,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR1_6() --------------------------
def FMR1_6():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 59 + y
    ID12 = 160- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,268,speed,2,821,speed,9,465,speed,10,530,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(196),toMXspeed(speed),14,toMX(756),toMXspeed(speed),15,toMX(669),toMXspeed(speed),16,toMX(390),toMXspeed(speed),17,479,speed,18,490,speed]
        else:
            goalPoss = [1,268,speed,2,821,speed,9,465,speed,10,530,speed,11,256+ID11,speed,12,512+ID12,speed,13,196,speed,14,756,speed,15,669,speed,16,390,speed,17,479,speed,18,490,speed]
    else :
        goalPoss = [9,465,speed,10,530,speed,11,256+ID11,speed,12,512+ID12,speed,13,196,speed,14,756,speed,15,669,speed,16,390,speed,17,479,speed,18,490,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_0() --------------------------
def FMR2_0():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 47 + y
    ID12 = 158- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,254,speed,2,807,speed,9,467,speed,10,527,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(203),toMXspeed(speed),14,toMX(757),toMXspeed(speed),15,toMX(649),toMXspeed(speed),16,toMX(387),toMXspeed(speed),17,480,speed,18,491,speed]
        else:
            goalPoss = [1,254,speed,2,807,speed,9,467,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,203,speed,14,757,speed,15,649,speed,16,387,speed,17,480,speed,18,491,speed]
    else:
        goalPoss = [9,467,speed,10,527,speed,11,256+ID11,speed,12,512+ID12,speed,13,203,speed,14,757,speed,15,649,speed,16,387,speed,17,480,speed,18,491,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_1() --------------------------
def FMR2_1():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 43 + y
    ID12 = 157- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,239,speed,2,792,speed,9,473,speed,10,519,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(222),toMXspeed(speed),14,toMX(761),toMXspeed(speed),15,toMX(627),toMXspeed(speed),16,toMX(382),toMXspeed(speed),17,482,speed,18,493,speed]
        else:
            goalPoss = [1,239,speed,2,792,speed,9,473,speed,10,519,speed,11,256+ID11,speed,12,512+ID12,speed,13,222,speed,14,761,speed,15,627,speed,16,382,speed,17,482,speed,18,493,speed]
    else:
        goalPoss = [9,473,speed,10,519,speed,11,256+ID11,speed,12,512+ID12,speed,13,222,speed,14,761,speed,15,627,speed,16,382,speed,17,482,speed,18,493,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_2() --------------------------
def FMR2_2():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 46 + y
    ID12 = 156- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,223,speed,2,776,speed,9,482,speed,10,510,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(245),toMXspeed(speed),14,toMX(766),toMXspeed(speed),15,toMX(607),toMXspeed(speed),16,toMX(376),toMXspeed(speed),17,486,speed,18,496,speed]
        else:
            goalPoss = [1,223,speed,2,776,speed,9,482,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,245,speed,14,766,speed,15,607,speed,16,376,speed,17,486,speed,18,496,speed]
    else:
        goalPoss = [9,482,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,245,speed,14,766,speed,15,607,speed,16,376,speed,17,486,speed,18,496,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_3() --------------------------
def FMR2_3():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 51 + y
    ID12 = 155- y  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,209,speed,2,762,speed,9,490,speed,10,504,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(262),toMXspeed(speed),14,toMX(771),toMXspeed(speed),15,toMX(595),toMXspeed(speed),16,toMX(369),toMXspeed(speed),17,491,speed,18,500,speed]
        else:
            goalPoss = [1,209,speed,2,762,speed,9,490,speed,10,504,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,771,speed,15,595,speed,16,369,speed,17,491,speed,18,500,speed]
    else :
        goalPoss = [9,490,speed,10,504,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,771,speed,15,595,speed,16,369,speed,17,491,speed,18,500,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_4() --------------------------
def FMR2_4():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 53 + (y)
    ID12 = 152- (y)  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,195,speed,2,748,speed,9,496,speed,10,505,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(262),toMXspeed(speed),14,toMX(775),toMXspeed(speed),15,toMX(596),toMXspeed(speed),16,toMX(363),toMXspeed(speed),17,496,speed,18,505,speed]
        else:
            goalPoss = [1,195,speed,2,748,speed,9,496,speed,10,505,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,775,speed,15,596,speed,16,363,speed,17,496,speed,18,505,speed]
    else:
        goalPoss = [9,496,speed,10,505,speed,11,256+ID11,speed,12,512+ID12,speed,13,262,speed,14,775,speed,15,596,speed,16,363,speed,17,496,speed,18,505,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_5() --------------------------
def FMR2_5():
    global full_body
    global y
    global speed
    global verMX

    ID11 = 53 + (y)
    ID12 = 147- (y)  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,184,speed,2,737,speed,9,501,speed,10,510,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(254),toMXspeed(speed),14,toMX(777),toMXspeed(speed),15,toMX(605),toMXspeed(speed),16,toMX(357),toMXspeed(speed),17,501,speed,18,510,speed]
        else:
            goalPoss = [1,184,speed,2,737,speed,9,501,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,254,speed,14,777,speed,15,605,speed,16,357,speed,17,501,speed,18,510,speed]
    else :
        goalPoss = [9,501,speed,10,510,speed,11,256+ID11,speed,12,512+ID12,speed,13,254,speed,14,777,speed,15,605,speed,16,357,speed,17,501,speed,18,510,speed]
    write_goalPos(goalPoss)
    msleep(37)

#------------------ void FMR2_6() --------------------------
def FMR2_6():
    global full_body
    global y
    global speed
    global verMX
    global fmr2_6_

    ID11 = 57 + (y)
    ID12 = 141- (y)  
    
    if full_body == 1:
        if verMX==1:
            goalPoss = [1,175,speed,2,728,speed,9,507,speed,10,516,speed,11,toMX(256+ID11),toMXspeed(speed),12,toMX(512+ID12),toMXspeed(speed),13,toMX(248),toMXspeed(speed),14,toMX(775),toMXspeed(speed),15,toMX(614),toMXspeed(speed),16,toMX(352),toMXspeed(speed),17,507,speed,18,516,speed]
        else:
            goalPoss = [1,175,speed,2,728,speed,9,507,speed,10,516,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,775,speed,15,614,speed,16,352,speed,17,507,speed,18,516,speed]
    else :
        goalPoss = [9,507,speed,10,516,speed,11,256+ID11,speed,12,512+ID12,speed,13,248,speed,14,775,speed,15,614,speed,16,352,speed,17,507,speed,18,516,speed]
    write_goalPos(goalPoss)
    msleep(37)
    fmr2_6_ = 1


#================= void RTML()   =============================
def RTML():
    #----------------- RTML() step 0 ----------------------------- 
    global full_body 
    global speed_muter_kanan
    
    goalPoss = [7,419,speed_muter_kanan,8,604,speed_muter_kanan,9,513,speed_muter_kanan,10,522,speed_muter_kanan,11,343,speed_muter_kanan,12,684,speed_muter_kanan,13,241,speed_muter_kanan,14,781,speed_muter_kanan,15,647,speed_muter_kanan,16,381,speed_muter_kanan,17,513,speed_muter_kanan,18,522,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML() step 1 -----------------------------  
    goalPoss = [9,519,speed_muter_kanan,10,527,speed_muter_kanan,11,346,speed_muter_kanan,12,683,speed_muter_kanan,13,245,speed_muter_kanan,14,776,speed_muter_kanan,15,647,speed_muter_kanan,16,385,speed_muter_kanan,17,519,speed_muter_kanan,18,527,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)
    
    #----------------- RTML() step 2 -----------------------------  
    goalPoss = [7,419,speed_muter_kanan,8,604,speed_muter_kanan,9,513,speed_muter_kanan,10,522,speed_muter_kanan,11,343,speed_muter_kanan,12,684,speed_muter_kanan,13,241,speed_muter_kanan,14,781,speed_muter_kanan,15,647,speed_muter_kanan,16,381,speed_muter_kanan,17,513,speed_muter_kanan,18,522,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML() step 3 -----------------------------  
    goalPoss = [7,413,speed_muter_kanan,8,610,speed_muter_kanan,9,515,speed_muter_kanan,10,543,speed_muter_kanan,11,355,speed_muter_kanan,12,693,speed_muter_kanan,13,257,speed_muter_kanan,14,790,speed_muter_kanan,15,643,speed_muter_kanan,16,380,speed_muter_kanan,17,529,speed_muter_kanan,18,538,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML() step 4 -----------------------------  
    goalPoss = [7,406,speed_muter_kanan,8,617,speed_muter_kanan,9,506,speed_muter_kanan,10,551,speed_muter_kanan,11,358,speed_muter_kanan,12,702,speed_muter_kanan,13,263,speed_muter_kanan,14,807,speed_muter_kanan,15,641,speed_muter_kanan,16,372,speed_muter_kanan,17,532,speed_muter_kanan,18,542,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML() step 5 -----------------------------  
    goalPoss = [7,398,speed_muter_kanan,8,625,speed_muter_kanan,9,498,speed_muter_kanan,10,558,speed_muter_kanan,11,360,speed_muter_kanan,12,708,speed_muter_kanan,13,267,speed_muter_kanan,14,821,speed_muter_kanan,15,638,speed_muter_kanan,16,365,speed_muter_kanan,17,534,speed_muter_kanan,18,546,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML() step 6 -----------------------------  
    goalPoss = [7,388,speed_muter_kanan,8,635,speed_muter_kanan,9,495,speed_muter_kanan,10,561,speed_muter_kanan,11,359,speed_muter_kanan,12,709,speed_muter_kanan,13,269,speed_muter_kanan,14,826,speed_muter_kanan,15,636,speed_muter_kanan,16,361,speed_muter_kanan,17,535,speed_muter_kanan,18,547,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 0 -----------------------------  
    goalPoss = [7,379,speed_muter_kanan,8,644,speed_muter_kanan,9,498,speed_muter_kanan,10,559,speed_muter_kanan,11,357,speed_muter_kanan,12,704,speed_muter_kanan,13,267,speed_muter_kanan,14,821,speed_muter_kanan,15,636,speed_muter_kanan,16,361,speed_muter_kanan,17,535,speed_muter_kanan,18,546,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 1 -----------------------------  
    goalPoss = [7,370,speed_muter_kanan,8,653,speed_muter_kanan,9,506,speed_muter_kanan,10,552,speed_muter_kanan,11,354,speed_muter_kanan,12,696,speed_muter_kanan,13,263,speed_muter_kanan,14,807,speed_muter_kanan,15,637,speed_muter_kanan,16,366,speed_muter_kanan,17,533,speed_muter_kanan,18,543,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 2 -----------------------------  
    goalPoss = [7,363,speed_muter_kanan,8,660,speed_muter_kanan,9,516,speed_muter_kanan,10,544,speed_muter_kanan,11,350,speed_muter_kanan,12,686,speed_muter_kanan,13,257,speed_muter_kanan,14,790,speed_muter_kanan,15,639,speed_muter_kanan,16,373,speed_muter_kanan,17,529,speed_muter_kanan,18,539,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 3 -----------------------------  
    goalPoss = [7,359,speed_muter_kanan,8,664,speed_muter_kanan,9,521,speed_muter_kanan,10,535,speed_muter_kanan,11,347,speed_muter_kanan,12,679,speed_muter_kanan,13,250,speed_muter_kanan,14,777,speed_muter_kanan,15,642,speed_muter_kanan,16,379,speed_muter_kanan,17,525,speed_muter_kanan,18,534,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 4 -----------------------------  
    goalPoss = [7,358,speed_muter_kanan,8,666,speed_muter_kanan,9,520,speed_muter_kanan,10,528,speed_muter_kanan,11,344,speed_muter_kanan,12,678,speed_muter_kanan,13,245,speed_muter_kanan,14,776,speed_muter_kanan,15,644,speed_muter_kanan,16,380,speed_muter_kanan,17,520,speed_muter_kanan,18,528,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 5 -----------------------------  
    goalPoss = [9,514,speed_muter_kanan,10,522,speed_muter_kanan,11,342,speed_muter_kanan,12,681,speed_muter_kanan,13,241,speed_muter_kanan,14,781,speed_muter_kanan,15,646,speed_muter_kanan,16,377,speed_muter_kanan,17,514,speed_muter_kanan,18,522,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTML2() step 6 -----------------------------  
    goalPoss = [9,507,speed_muter_kanan,10,516,speed_muter_kanan,11,341,speed_muter_kanan,12,682,speed_muter_kanan,13,240,speed_muter_kanan,14,783,speed_muter_kanan,15,647,speed_muter_kanan,16,376,speed_muter_kanan,17,507,speed_muter_kanan,18,516,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)


#================= void RTMR()   =============================
def RTMR():
    #----------------- RTMR() step 0 -----------------------------  
    global full_body
    global speed_muter_kanan

    goalPoss = [7,358,speed_muter_kanan,8,666,speed_muter_kanan,9,501,speed_muter_kanan,10,509,speed_muter_kanan,11,342,speed_muter_kanan,12,681,speed_muter_kanan,13,242,speed_muter_kanan,14,782,speed_muter_kanan,15,646,speed_muter_kanan,16,377,speed_muter_kanan,17,501,speed_muter_kanan,18,509,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR() step 1 -----------------------------  
    goalPoss = [9,495,speed_muter_kanan,10,503,speed_muter_kanan,11,345,speed_muter_kanan,12,679,speed_muter_kanan,13,247,speed_muter_kanan,14,778,speed_muter_kanan,15,643,speed_muter_kanan,16,379,speed_muter_kanan,17,495,speed_muter_kanan,18,503,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR() step 2 -----------------------------  
    goalPoss = [7,359,speed_muter_kanan,8,664,speed_muter_kanan,9,488,speed_muter_kanan,10,502,speed_muter_kanan,11,344,speed_muter_kanan,12,676,speed_muter_kanan,13,246,speed_muter_kanan,14,773,speed_muter_kanan,15,644,speed_muter_kanan,16,381,speed_muter_kanan,17,489,speed_muter_kanan,18,498,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR() step 3 -----------------------------  
    goalPoss = [7,363,speed_muter_kanan,8,404,speed_muter_kanan,9,479,speed_muter_kanan,10,507,speed_muter_kanan,11,337,speed_muter_kanan,12,673,speed_muter_kanan,13,233,speed_muter_kanan,14,766,speed_muter_kanan,15,650,speed_muter_kanan,16,384,speed_muter_kanan,17,484,speed_muter_kanan,18,494,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR() step 4 -----------------------------  
    goalPoss = [7,370,speed_muter_kanan,8,653,speed_muter_kanan,9,471,speed_muter_kanan,10,517,speed_muter_kanan,11,327,speed_muter_kanan,12,669,speed_muter_kanan,13,216,speed_muter_kanan,14,760,speed_muter_kanan,15,657,speed_muter_kanan,16,386,speed_muter_kanan,17,480,speed_muter_kanan,18,490,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR() step 5 -----------------------------  
    goalPoss = [7,379,speed_muter_kanan,8,644,speed_muter_kanan,9,464,speed_muter_kanan,10,525,speed_muter_kanan,11,319,speed_muter_kanan,12,666,speed_muter_kanan,13,202,speed_muter_kanan,14,756,speed_muter_kanan,15,662,speed_muter_kanan,16,387,speed_muter_kanan,17,477,speed_muter_kanan,18,488,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR() step 6 -----------------------------  
    goalPoss = [7,388,speed_muter_kanan,8,635,speed_muter_kanan,9,462,speed_muter_kanan,10,528,speed_muter_kanan,11,314,speed_muter_kanan,12,664,speed_muter_kanan,13,197,speed_muter_kanan,14,754,speed_muter_kanan,17,476,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 0 -----------------------------  
    goalPoss = [7,398,speed_muter_kanan,8,625,speed_muter_kanan,9,465,speed_muter_kanan,10,525,speed_muter_kanan,11,315,speed_muter_kanan,12,663,speed_muter_kanan,13,202,speed_muter_kanan,14,756,speed_muter_kanan,15,658,speed_muter_kanan,16,385,speed_muter_kanan,17,477,speed_muter_kanan,18,489,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 1 -----------------------------  
    goalPoss = [7,406,speed_muter_kanan,8,617,speed_muter_kanan,9,472,speed_muter_kanan,10,517,speed_muter_kanan,11,321,speed_muter_kanan,12,665,speed_muter_kanan,13,216,speed_muter_kanan,14,760,speed_muter_kanan,15,651,speed_muter_kanan,16,382,speed_muter_kanan,17,481,speed_muter_kanan,18,491,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 2 -----------------------------  
    goalPoss = [7,413,speed_muter_kanan,8,610,speed_muter_kanan,9,480,speed_muter_kanan,10,508,speed_muter_kanan,11,330,speed_muter_kanan,12,668,speed_muter_kanan,13,233,speed_muter_kanan,14,766,speed_muter_kanan,15,643,speed_muter_kanan,16,380,speed_muter_kanan,17,485,speed_muter_kanan,18,494,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 3 -----------------------------  
    goalPoss = [7,417,speed_muter_kanan,8,606,speed_muter_kanan,9,489,speed_muter_kanan,10,502,speed_muter_kanan,11,337,speed_muter_kanan,12,673,speed_muter_kanan,13,246,speed_muter_kanan,14,773,speed_muter_kanan,15,637,speed_muter_kanan,16,377,speed_muter_kanan,17,490,speed_muter_kanan,18,499,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 4 -----------------------------  
    goalPoss = [7,430,speed_muter_kanan,8,604,speed_muter_kanan,9,496,speed_muter_kanan,10,504,speed_muter_kanan,11,340,speed_muter_kanan,12,677,speed_muter_kanan,13,247,speed_muter_kanan,14,778,speed_muter_kanan,15,638,speed_muter_kanan,16,376,speed_muter_kanan,17,496,speed_muter_kanan,18,504,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 5 -----------------------------  
    goalPoss = [9,501,speed_muter_kanan,10,510,speed_muter_kanan,11,339,speed_muter_kanan,12,680,speed_muter_kanan,13,242,speed_muter_kanan,14,782,speed_muter_kanan,15,642,speed_muter_kanan,16,376,speed_muter_kanan,17,501,speed_muter_kanan,18,510,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)

    #----------------- RTMR2() step 6 -----------------------------  
    goalPoss = [9,507,speed_muter_kanan,10,516,speed_muter_kanan,11,340,speed_muter_kanan,12,683,speed_muter_kanan,13,240,speed_muter_kanan,14,783,speed_muter_kanan,15,645,speed_muter_kanan,16,378,speed_muter_kanan,17,507,speed_muter_kanan,18,516,speed_muter_kanan]
    write_goalPos(goalPoss)
    msleep(25)


#================= void LTML()   =============================
def LTML():
    #----------------- LTML() step 0 -----------------------------  
    global full_body
    global speed_muter_kiri

    if full_body == 1:
        goalPoss = [1,235,speed_muter_kiri,2,788,speed_muter_kiri,3,279,speed_muter_kiri,4,744,speed_muter_kiri,5,462,speed_muter_kiri,6,561,speed_muter_kiri,7,358,speed_muter_kiri,8,666,speed_muter_kiri,9,514,speed_muter_kiri,10,522,speed_muter_kiri,11,342,speed_muter_kiri,12,681,speed_muter_kiri,13,241,speed_muter_kiri,14,781,speed_muter_kiri,15,646,speed_muter_kiri,16,377,speed_muter_kiri,17,514,speed_muter_kiri,18,522,speed_muter_kiri]
    else :
        goalPoss = [7,358,speed_muter_kiri,8,666,speed_muter_kiri,9,514,speed_muter_kiri,10,522,speed_muter_kiri,11,342,speed_muter_kiri,12,681,speed_muter_kiri,13,241,speed_muter_kiri,14,781,speed_muter_kiri,15,646,speed_muter_kiri,16,377,speed_muter_kiri,17,514,speed_muter_kiri,18,522,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- LTML() step 1 -----------------------------  
    goalPoss = [9,520,speed_muter_kiri,10,528,speed_muter_kiri,11,344,speed_muter_kiri,12,678,speed_muter_kiri,13,245,speed_muter_kiri,14,776,speed_muter_kiri,15,644,speed_muter_kiri,16,380,speed_muter_kiri,17,520,speed_muter_kiri,18,528,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- LTML() step 2 -----------------------------  
    goalPoss = [7,359,speed_muter_kiri,8,664,speed_muter_kiri,9,521,speed_muter_kiri,10,535,speed_muter_kiri,11,347,speed_muter_kiri,12,679,speed_muter_kiri,13,250,speed_muter_kiri,14,777,speed_muter_kiri,15,642,speed_muter_kiri,16,379,speed_muter_kiri,17,525,speed_muter_kiri,18,534,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- LTML() step 3 -----------------------------  
    goalPoss = [7,363,speed_muter_kiri,8,660,speed_muter_kiri,9,516,speed_muter_kiri,10,544,speed_muter_kiri,11,350,speed_muter_kiri,12,686,speed_muter_kiri,13,257,speed_muter_kiri,14,790,speed_muter_kiri,15,639,speed_muter_kiri,16,373,speed_muter_kiri,17,529,speed_muter_kiri,18,539,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- LTML() step 4 -----------------------------  
    goalPoss = [7,370,speed_muter_kiri,8,653,speed_muter_kiri,9,506,speed_muter_kiri,10,552,speed_muter_kiri,11,354,speed_muter_kiri,12,696,speed_muter_kiri,13,263,speed_muter_kiri,14,807,speed_muter_kiri,15,637,speed_muter_kiri,16,366,speed_muter_kiri,17,533,speed_muter_kiri,18,543,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- LTML() step 5 -----------------------------  
    goalPoss = [7,379,speed_muter_kiri,8,644,speed_muter_kiri,9,498,speed_muter_kiri,10,559,speed_muter_kiri,11,357,speed_muter_kiri,12,704,speed_muter_kiri,13,267,speed_muter_kiri,14,821,speed_muter_kiri,15,636,speed_muter_kiri,16,361,speed_muter_kiri,17,535,speed_muter_kiri,18,546,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- LTML() step 6 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,495,speed_muter_kiri,10,561,speed_muter_kiri,11,359,speed_muter_kiri,12,709,speed_muter_kiri,13,269,speed_muter_kiri,14,826,speed_muter_kiri,15,636,speed_muter_kiri,16,361,speed_muter_kiri,17,535,speed_muter_kiri,18,547,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 0 -----------------------------  
    goalPoss = [7,398,speed_muter_kiri,8,625,speed_muter_kiri,9,498,speed_muter_kiri,10,558,speed_muter_kiri,11,360,speed_muter_kiri,12,708,speed_muter_kiri,13,267,speed_muter_kiri,14,821,speed_muter_kiri,15,638,speed_muter_kiri,16,365,speed_muter_kiri,17,534,speed_muter_kiri,18,546,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 1 -----------------------------  
    goalPoss = [7,406,speed_muter_kiri,8,617,speed_muter_kiri,9,506,speed_muter_kiri,10,551,speed_muter_kiri,11,358,speed_muter_kiri,12,702,speed_muter_kiri,13,263,speed_muter_kiri,14,807,speed_muter_kiri,15,641,speed_muter_kiri,16,372,speed_muter_kiri,17,532,speed_muter_kiri,18,542,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 2 -----------------------------  
    goalPoss = [7,413,speed_muter_kiri,8,610,speed_muter_kiri,9,515,speed_muter_kiri,10,543,speed_muter_kiri,11,355,speed_muter_kiri,12,693,speed_muter_kiri,13,257,speed_muter_kiri,14,790,speed_muter_kiri,15,643,speed_muter_kiri,16,380,speed_muter_kiri,17,529,speed_muter_kiri,18,538,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 3 -----------------------------  
    goalPoss = [7,417,speed_muter_kiri,8,606,speed_muter_kiri,9,521,speed_muter_kiri,10,534,speed_muter_kiri,11,350,speed_muter_kiri,12,686,speed_muter_kiri,13,250,speed_muter_kiri,14,777,speed_muter_kiri,15,646,speed_muter_kiri,16,386,speed_muter_kiri,17,524,speed_muter_kiri,18,533,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 4 -----------------------------  
    goalPoss = [7,419,speed_muter_kiri,8,604,speed_muter_kiri,9,519,speed_muter_kiri,10,527,speed_muter_kiri,11,346,speed_muter_kiri,12,683,speed_muter_kiri,13,245,speed_muter_kiri,14,776,speed_muter_kiri,15,647,speed_muter_kiri,16,385,speed_muter_kiri,17,519,speed_muter_kiri,18,527,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 5 -----------------------------  
    goalPoss = [9,513,speed_muter_kiri,10,522,speed_muter_kiri,11,343,speed_muter_kiri,12,684,speed_muter_kiri,13,241,speed_muter_kiri,14,781,speed_muter_kiri,15,647,speed_muter_kiri,16,381,speed_muter_kiri,17,513,speed_muter_kiri,18,522,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #----------------- RTML2() step 6 -----------------------------  
    goalPoss = [9,507,speed_muter_kiri,10,516,speed_muter_kiri,11,340,speed_muter_kiri,12,683,speed_muter_kiri,13,240,speed_muter_kiri,14,783,speed_muter_kiri,15,645,speed_muter_kiri,16,378,speed_muter_kiri,17,507,speed_muter_kiri,18,516,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)


#=================== void LTMR() ================================
def LTMR():
    #-------------------- LTMR() step 0 -----------------------------  
    global full_body
    global speed_muter_kiri

    if full_body == 1:
        goalPoss = [1,235,speed_muter_kiri,2,788,speed_muter_kiri,3,279,speed_muter_kiri,4,744,speed_muter_kiri,5,462,speed_muter_kiri,6,561,speed_muter_kiri,7,419,speed_muter_kiri,8,604,speed_muter_kiri,9,501,speed_muter_kiri,10,510,speed_muter_kiri,11,339,speed_muter_kiri,12,680,speed_muter_kiri,13,242,speed_muter_kiri,14,782,speed_muter_kiri,15,642,speed_muter_kiri,16,376,speed_muter_kiri,17,501,speed_muter_kiri,18,510,speed_muter_kiri]
    else :
        goalPoss = [7,419,speed_muter_kiri,8,604,speed_muter_kiri,9,501,speed_muter_kiri,10,510,speed_muter_kiri,11,339,speed_muter_kiri,12,680,speed_muter_kiri,13,242,speed_muter_kiri,14,782,speed_muter_kiri,15,642,speed_muter_kiri,16,376,speed_muter_kiri,17,501,speed_muter_kiri,18,510,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR() step 1 -----------------------------  
    goalPoss = [9,496,speed_muter_kiri,10,504,speed_muter_kiri,11,340,speed_muter_kiri,12,677,speed_muter_kiri,13,247,speed_muter_kiri,14,778,speed_muter_kiri,15,638,speed_muter_kiri,16,376,speed_muter_kiri,17,496,speed_muter_kiri,18,504,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR() step 2 -----------------------------  
    goalPoss = [7,417,speed_muter_kiri,8,606,speed_muter_kiri,9,489,speed_muter_kiri,10,502,speed_muter_kiri,11,337,speed_muter_kiri,12,673,speed_muter_kiri,13,246,speed_muter_kiri,14,773,speed_muter_kiri,15,637,speed_muter_kiri,16,377,speed_muter_kiri,17,490,speed_muter_kiri,18,499,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR() step 3 -----------------------------  
    goalPoss = [7,413,speed_muter_kiri,8,610,speed_muter_kiri,9,480,speed_muter_kiri,10,508,speed_muter_kiri,11,330,speed_muter_kiri,12,668,speed_muter_kiri,13,233,speed_muter_kiri,14,766,speed_muter_kiri,15,643,speed_muter_kiri,16,380,speed_muter_kiri,17,485,speed_muter_kiri,18,494,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR() step 4 -----------------------------  
    goalPoss = [7,406,speed_muter_kiri,8,617,speed_muter_kiri,9,472,speed_muter_kiri,10,517,speed_muter_kiri,11,321,speed_muter_kiri,12,665,speed_muter_kiri,13,216,speed_muter_kiri,14,760,speed_muter_kiri,15,651,speed_muter_kiri,16,382,speed_muter_kiri,17,481,speed_muter_kiri,18,491,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR() step 5 -----------------------------  
    goalPoss = [7,398,speed_muter_kiri,8,625,speed_muter_kiri,9,465,speed_muter_kiri,10,525,speed_muter_kiri,11,315,speed_muter_kiri,12,663,speed_muter_kiri,13,202,speed_muter_kiri,14,756,speed_muter_kiri,15,658,speed_muter_kiri,16,385,speed_muter_kiri,17,477,speed_muter_kiri,18,489,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR() step 6 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 0 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 1 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 2 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 3 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 4 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 5 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)

    #-------------------- LTMR2() step 6 -----------------------------  
    goalPoss = [7,388,speed_muter_kiri,8,635,speed_muter_kiri,9,462,speed_muter_kiri,10,528,speed_muter_kiri,11,314,speed_muter_kiri,12,664,speed_muter_kiri,13,197,speed_muter_kiri,14,754,speed_muter_kiri,15,662,speed_muter_kiri,16,387,speed_muter_kiri,17,476,speed_muter_kiri,18,488,speed_muter_kiri]
    write_goalPos(goalPoss)
    msleep(26)


#================= void start_kanan() //RSR ====================
def start_kanan():
    #--------------------- RSR step 0 ------------------------------
    global full_body
    global speed_awal_samping_kanan

    if full_body == 1:
        goalPoss = [1,235,speed_awal_samping_kanan,2,788,speed_awal_samping_kanan,3,279,speed_awal_samping_kanan,4,744,speed_awal_samping_kanan,5,462,speed_awal_samping_kanan,6,561,speed_awal_samping_kanan,7,358,speed_awal_samping_kanan,8,666,speed_awal_samping_kanan,9,501,speed_awal_samping_kanan,10,510,speed_awal_samping_kanan,11,342,speed_awal_samping_kanan,12,681,speed_awal_samping_kanan,13,242,speed_awal_samping_kanan,14,782,speed_awal_samping_kanan,15,646,speed_awal_samping_kanan,16,377,speed_awal_samping_kanan,17,501,speed_awal_samping_kanan,18,510,speed_awal_samping_kanan]
    else :
        goalPoss = [7,358,speed_awal_samping_kanan,8,666,speed_awal_samping_kanan,9,501,speed_awal_samping_kanan,10,510,speed_awal_samping_kanan,11,342,speed_awal_samping_kanan,12,681,speed_awal_samping_kanan,13,242,speed_awal_samping_kanan,14,782,speed_awal_samping_kanan,15,646,speed_awal_samping_kanan,16,377,speed_awal_samping_kanan,17,501,speed_awal_samping_kanan,18,510,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR step 1 ------------------------------
    goalPoss = [9,495,speed_awal_samping_kanan,10,504,speed_awal_samping_kanan,11,345,speed_awal_samping_kanan,12,679,speed_awal_samping_kanan,13,247,speed_awal_samping_kanan,14,778,speed_awal_samping_kanan,15,643,speed_awal_samping_kanan,16,379,speed_awal_samping_kanan,17,495,speed_awal_samping_kanan,18,504,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR step 2 ------------------------------
    goalPoss = [9,490,speed_awal_samping_kanan,10,499,speed_awal_samping_kanan,11,348,speed_awal_samping_kanan,12,677,speed_awal_samping_kanan,13,253,speed_awal_samping_kanan,14,773,speed_awal_samping_kanan,15,640,speed_awal_samping_kanan,16,381,speed_awal_samping_kanan,17,490,speed_awal_samping_kanan,18,499,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR step 3 ------------------------------
    goalPoss = [9,486,speed_awal_samping_kanan,10,495,speed_awal_samping_kanan,11,351,speed_awal_samping_kanan,12,674,speed_awal_samping_kanan,13,260,speed_awal_samping_kanan,14,767,speed_awal_samping_kanan,15,637,speed_awal_samping_kanan,16,384,speed_awal_samping_kanan,17,486,speed_awal_samping_kanan,18,495,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR step 4 ------------------------------
    goalPoss = [9,479,speed_awal_samping_kanan,10,502,speed_awal_samping_kanan,11,344,speed_awal_samping_kanan,12,671,speed_awal_samping_kanan,13,245,speed_awal_samping_kanan,14,761,speed_awal_samping_kanan,15,644,speed_awal_samping_kanan,16,387,speed_awal_samping_kanan,17,482,speed_awal_samping_kanan,18,492,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR step 5 ------------------------------
    goalPoss = [9,468,speed_awal_samping_kanan,10,520,speed_awal_samping_kanan,11,327,speed_awal_samping_kanan,12,669,speed_awal_samping_kanan,13,212,speed_awal_samping_kanan,14,757,speed_awal_samping_kanan,15,661,speed_awal_samping_kanan,16,389,speed_awal_samping_kanan,17,479,speed_awal_samping_kanan,18,490,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR step 6 ------------------------------
    goalPoss = [9,463,speed_awal_samping_kanan,10,529,speed_awal_samping_kanan,11,320,speed_awal_samping_kanan,12,668,speed_awal_samping_kanan,13,197,speed_awal_samping_kanan,14,755,speed_awal_samping_kanan,15,668,speed_awal_samping_kanan,16,390,speed_awal_samping_kanan,17,477,speed_awal_samping_kanan,18,489,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 0 -----------------------------  
    goalPoss = [9,427,speed_awal_samping_kanan,10,531,speed_awal_samping_kanan,11,348,speed_awal_samping_kanan,12,671,speed_awal_samping_kanan,13,253,speed_awal_samping_kanan,14,761,speed_awal_samping_kanan,15,640,speed_awal_samping_kanan,16,387,speed_awal_samping_kanan,17,437,speed_awal_samping_kanan,18,501,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 1 -----------------------------  
    goalPoss = [9,434,speed_awal_samping_kanan,10,521,speed_awal_samping_kanan,11,371,speed_awal_samping_kanan,12,673,speed_awal_samping_kanan,13,300,speed_awal_samping_kanan,14,765,speed_awal_samping_kanan,15,617,speed_awal_samping_kanan,16,385,speed_awal_samping_kanan,17,438,speed_awal_samping_kanan,18,511,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 2 -----------------------------  
    goalPoss = [9,441,speed_awal_samping_kanan,10,518,speed_awal_samping_kanan,11,379,speed_awal_samping_kanan,12,675,speed_awal_samping_kanan,13,316,speed_awal_samping_kanan,14,769,speed_awal_samping_kanan,15,609,speed_awal_samping_kanan,16,383,speed_awal_samping_kanan,17,441,speed_awal_samping_kanan,18,518,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 3 -----------------------------  
    goalPoss = [9,447,speed_awal_samping_kanan,10,525,speed_awal_samping_kanan,11,370,speed_awal_samping_kanan,12,677,speed_awal_samping_kanan,13,297,speed_awal_samping_kanan,14,773,speed_awal_samping_kanan,15,618,speed_awal_samping_kanan,16,381,speed_awal_samping_kanan,17,447,speed_awal_samping_kanan,18,525,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 4 -----------------------------  
    goalPoss = [9,454,speed_awal_samping_kanan,10,533,speed_awal_samping_kanan,11,361,speed_awal_samping_kanan,12,677,speed_awal_samping_kanan,13,280,speed_awal_samping_kanan,14,774,speed_awal_samping_kanan,15,627,speed_awal_samping_kanan,16,381,speed_awal_samping_kanan,17,454,speed_awal_samping_kanan,18,533,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 5 -----------------------------  
    goalPoss = [9,462,speed_awal_samping_kanan,10,542,speed_awal_samping_kanan,11,354,speed_awal_samping_kanan,12,677,speed_awal_samping_kanan,13,266,speed_awal_samping_kanan,14,773,speed_awal_samping_kanan,15,634,speed_awal_samping_kanan,16,381,speed_awal_samping_kanan,17,462,speed_awal_samping_kanan,18,542,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- RSR2 step 6 -----------------------------  
    goalPoss = [9,472,speed_awal_samping_kanan,10,551,speed_awal_samping_kanan,11,349,speed_awal_samping_kanan,12,674,speed_awal_samping_kanan,13,256,speed_awal_samping_kanan,14,767,speed_awal_samping_kanan,15,639,speed_awal_samping_kanan,16,384,speed_awal_samping_kanan,17,472,speed_awal_samping_kanan,18,551,speed_awal_samping_kanan]
    write_goalPos(goalPoss)
    msleep(41)


#==================== void RML()  ==============================
def RML():
    #--------------------- RML step 0 ------------------------------
    global full_body
    global speed_jalan_samping

    if full_body == 1:
        goalPoss = [1,235,speed_jalan_samping,2,788,speed_jalan_samping,3,279,speed_jalan_samping,4,744,speed_jalan_samping,5,462,speed_jalan_samping,6,561,speed_jalan_samping,7,358,speed_jalan_samping,8,666,speed_jalan_samping,9,479,speed_jalan_samping,10,559,speed_jalan_samping,11,344,speed_jalan_samping,12,673,speed_jalan_samping,13,251,speed_jalan_samping,14,759,speed_jalan_samping,15,641,speed_jalan_samping,16,388,speed_jalan_samping,17,479,speed_jalan_samping,18,559,speed_jalan_samping]
    else :
        goalPoss = [7,358,speed_jalan_samping,8,666,speed_jalan_samping,9,479,speed_jalan_samping,10,559,speed_jalan_samping,11,344,speed_jalan_samping,12,673,speed_jalan_samping,13,251,speed_jalan_samping,14,759,speed_jalan_samping,15,641,speed_jalan_samping,16,388,speed_jalan_samping,17,479,speed_jalan_samping,18,559,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML step 1 ------------------------------
    goalPoss = [9,487,speed_jalan_samping,10,566,speed_jalan_samping,11,343,speed_jalan_samping,12,667,speed_jalan_samping,13,250,speed_jalan_samping,14,747,speed_jalan_samping,15,642,speed_jalan_samping,16,394,speed_jalan_samping,17,487,speed_jalan_samping,18,566,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML step 2 ------------------------------
    goalPoss = [9,491,speed_jalan_samping,10,573,speed_jalan_samping,11,344,speed_jalan_samping,12,665,speed_jalan_samping,13,251,speed_jalan_samping,14,743,speed_jalan_samping,15,641,speed_jalan_samping,16,396,speed_jalan_samping,17,495,speed_jalan_samping,18,572,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML step 3 ------------------------------
    goalPoss = [9,489,speed_jalan_samping,10,581,speed_jalan_samping,11,345,speed_jalan_samping,12,671,speed_jalan_samping,13,254,speed_jalan_samping,14,756,speed_jalan_samping,15,640,speed_jalan_samping,16,390,speed_jalan_samping,17,503,speed_jalan_samping,18,576,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML step 4 ------------------------------
    goalPoss = [9,485,speed_jalan_samping,10,587,speed_jalan_samping,11,347,speed_jalan_samping,12,681,speed_jalan_samping,13,258,speed_jalan_samping,14,776,speed_jalan_samping,15,638,speed_jalan_samping,16,380,speed_jalan_samping,17,511,speed_jalan_samping,18,578,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML step 5 ------------------------------
    goalPoss = [9,482,speed_jalan_samping,10,590,speed_jalan_samping,11,349,speed_jalan_samping,12,691,speed_jalan_samping,13,262,speed_jalan_samping,14,795,speed_jalan_samping,15,636,speed_jalan_samping,16,370,speed_jalan_samping,17,519,speed_jalan_samping,18,577,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML step 6 ------------------------------
    goalPoss = [9,485,speed_jalan_samping,10,587,speed_jalan_samping,11,350,speed_jalan_samping,12,696,speed_jalan_samping,13,264,speed_jalan_samping,14,806,speed_jalan_samping,15,635,speed_jalan_samping,16,365,speed_jalan_samping,17,525,speed_jalan_samping,18,573,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 0 ------------------------------
    goalPoss = [9,493,speed_jalan_samping,10,579,speed_jalan_samping,11,351,speed_jalan_samping,12,696,speed_jalan_samping,13,265,speed_jalan_samping,14,806,speed_jalan_samping,15,634,speed_jalan_samping,16,365,speed_jalan_samping,17,529,speed_jalan_samping,18,566,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 1 ------------------------------
    goalPoss = [9,506,speed_jalan_samping,10,567,speed_jalan_samping,11,349,speed_jalan_samping,12,692,speed_jalan_samping,13,262,speed_jalan_samping,14,798,speed_jalan_samping,15,636,speed_jalan_samping,16,369,speed_jalan_samping,17,532,speed_jalan_samping,18,557,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 2 ------------------------------
    goalPoss = [9,518,speed_jalan_samping,10,553,speed_jalan_samping,11,347,speed_jalan_samping,12,686,speed_jalan_samping,13,258,speed_jalan_samping,14,785,speed_jalan_samping,15,638,speed_jalan_samping,16,375,speed_jalan_samping,17,532,speed_jalan_samping,18,548,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 3 ------------------------------
    goalPoss = [9,525,speed_jalan_samping,10,540,speed_jalan_samping,11,344,speed_jalan_samping,12,681,speed_jalan_samping,13,251,speed_jalan_samping,14,775,speed_jalan_samping,15,641,speed_jalan_samping,16,380,speed_jalan_samping,17,529,speed_jalan_samping,18,539,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 4 ------------------------------
    goalPoss = [9,267,speed_jalan_samping,10,531,speed_jalan_samping,11,341,speed_jalan_samping,12,681,speed_jalan_samping,13,245,speed_jalan_samping,14,775,speed_jalan_samping,15,644,speed_jalan_samping,16,380,speed_jalan_samping,17,267,speed_jalan_samping,18,531,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 5 ------------------------------
    goalPoss = [9,515,speed_jalan_samping,10,524,speed_jalan_samping,11,339,speed_jalan_samping,12,683,speed_jalan_samping,13,241,speed_jalan_samping,14,780,speed_jalan_samping,15,646,speed_jalan_samping,16,378,speed_jalan_samping,17,515,speed_jalan_samping,18,524,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RML2 step 6 ------------------------------
    goalPoss = [9,507,speed_jalan_samping,10,516,speed_jalan_samping,11,338,speed_jalan_samping,12,685,speed_jalan_samping,13,240,speed_jalan_samping,14,783,speed_jalan_samping,15,647,speed_jalan_samping,16,376,speed_jalan_samping,17,507,speed_jalan_samping,18,516,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)


#==================== void RMR()  ==============================
def RMR():
    #--------------------- RMR1 step 0 -----------------------------
    global full_body
    global speed_jalan_samping
    
    goalPoss = [9,499,speed_jalan_samping,10,508,speed_jalan_samping,11,340,speed_jalan_samping,12,684,speed_jalan_samping,13,243,speed_jalan_samping,14,782,speed_jalan_samping,15,645,speed_jalan_samping,16,377,speed_jalan_samping,17,499,speed_jalan_samping,18,508,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR1 step 1 -----------------------------
    goalPoss = [9,492,speed_jalan_samping,10,500,speed_jalan_samping,11,342,speed_jalan_samping,12,682,speed_jalan_samping,13,248,speed_jalan_samping,14,778,speed_jalan_samping,15,643,speed_jalan_samping,16,379,speed_jalan_samping,17,492,speed_jalan_samping,18,500,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR1 step 2 -----------------------------
    goalPoss = [9,483,speed_jalan_samping,10,498,speed_jalan_samping,11,342,speed_jalan_samping,12,679,speed_jalan_samping,13,248,speed_jalan_samping,14,772,speed_jalan_samping,15,643,speed_jalan_samping,16,382,speed_jalan_samping,17,484,speed_jalan_samping,18,494,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR1 step 3 -----------------------------
    goalPoss = [9,470,speed_jalan_samping,10,505,speed_jalan_samping,11,337,speed_jalan_samping,12,676,speed_jalan_samping,13,238,speed_jalan_samping,14,765,speed_jalan_samping,15,648,speed_jalan_samping,16,385,speed_jalan_samping,17,475,speed_jalan_samping,18,491,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR1 step 4 -----------------------------
    goalPoss = [9,456,speed_jalan_samping,10,517,speed_jalan_samping,11,331,speed_jalan_samping,12,674,speed_jalan_samping,13,225,speed_jalan_samping,14,761,speed_jalan_samping,15,654,speed_jalan_samping,16,387,speed_jalan_samping,17,466,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR1 step 5 -----------------------------
    goalPoss = [9,444,speed_jalan_samping,10,530,speed_jalan_samping,11,327,speed_jalan_samping,12,672,speed_jalan_samping,13,217,speed_jalan_samping,14,758,speed_jalan_samping,15,658,speed_jalan_samping,16,389,speed_jalan_samping,17,457,speed_jalan_samping,18,494,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR1 step 6 -----------------------------
    goalPoss = [9,436,speed_jalan_samping,10,538,speed_jalan_samping,11,327,speed_jalan_samping,12,673,speed_jalan_samping,14,759,speed_jalan_samping,16,384,speed_jalan_samping,17,450,speed_jalan_samping,18,498,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 0  ---------------------------
    goalPoss = [9,433,speed_jalan_samping,10,541,speed_jalan_samping,11,332,speed_jalan_samping,12,674,speed_jalan_samping,13,228,speed_jalan_samping,14,761,speed_jalan_samping,15,653,speed_jalan_samping,16,387,speed_jalan_samping,17,446,speed_jalan_samping,18,504,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 1  ---------------------------
    goalPoss = [9,436,speed_jalan_samping,10,538,speed_jalan_samping,11,342,speed_jalan_samping,12,676,speed_jalan_samping,13,247,speed_jalan_samping,14,765,speed_jalan_samping,15,643,speed_jalan_samping,16,385,speed_jalan_samping,17,445,speed_jalan_samping,18,512,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 2  ---------------------------
    goalPoss = [9,442,speed_jalan_samping,10,534,speed_jalan_samping,11,352,speed_jalan_samping,12,678,speed_jalan_samping,13,267,speed_jalan_samping,14,769,speed_jalan_samping,15,633,speed_jalan_samping,16,383,speed_jalan_samping,17,447,speed_jalan_samping,18,520,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 3  ---------------------------
    goalPoss = [9,450,speed_jalan_samping,10,532,speed_jalan_samping,11,358,speed_jalan_samping,12,679,speed_jalan_samping,13,280,speed_jalan_samping,14,772,speed_jalan_samping,15,627,speed_jalan_samping,16,382,speed_jalan_samping,17,451,speed_jalan_samping,18,528,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 4  ---------------------------
    goalPoss = [9,457,speed_jalan_samping,10,536,speed_jalan_samping,11,356,speed_jalan_samping,12,680,speed_jalan_samping,13,276,speed_jalan_samping,14,773,speed_jalan_samping,15,629,speed_jalan_samping,16,381,speed_jalan_samping,17,457,speed_jalan_samping,18,536,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 5  ---------------------------
    goalPoss = [9,464,speed_jalan_samping,10,544,speed_jalan_samping,11,350,speed_jalan_samping,12,679,speed_jalan_samping,13,264,speed_jalan_samping,14,772,speed_jalan_samping,15,635,speed_jalan_samping,16,382,speed_jalan_samping,17,464,speed_jalan_samping,18,544,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- RMR2 step 6  ---------------------------
    goalPoss = [9,472,speed_jalan_samping,10,551,speed_jalan_samping,11,346,speed_jalan_samping,12,677,speed_jalan_samping,13,256,speed_jalan_samping,14,767,speed_jalan_samping,15,639,speed_jalan_samping,16,384,speed_jalan_samping,17,472,speed_jalan_samping,18,551,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)


#===================== void start_kiri()  =======================
def start_kiri():
    #--------------------- start_kiri step 0  -----------------------
    global full_body
    global speed_awal_samping_kiri

    if full_body == 1:
        goalPoss = [1,235,speed_awal_samping_kiri,2,788,speed_awal_samping_kiri,3,279,speed_awal_samping_kiri,4,744,speed_awal_samping_kiri,5,462,speed_awal_samping_kiri,6,561,speed_awal_samping_kiri,7,358,speed_awal_samping_kiri,8,666,speed_awal_samping_kiri,9,515,speed_awal_samping_kiri,10,524,speed_awal_samping_kiri,11,342,speed_awal_samping_kiri,12,680,speed_awal_samping_kiri,13,241,speed_awal_samping_kiri,14,780,speed_awal_samping_kiri,15,646,speed_awal_samping_kiri,16,378,speed_awal_samping_kiri,17,515,speed_awal_samping_kiri,18,524,speed_awal_samping_kiri]
    else :
        goalPoss = [7,358,speed_awal_samping_kiri,8,666,speed_awal_samping_kiri,9,515,speed_awal_samping_kiri,10,524,speed_awal_samping_kiri,11,342,speed_awal_samping_kiri,12,680,speed_awal_samping_kiri,13,241,speed_awal_samping_kiri,14,780,speed_awal_samping_kiri,15,646,speed_awal_samping_kiri,16,378,speed_awal_samping_kiri,17,515,speed_awal_samping_kiri,18,524,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- start_kiri step 1  -----------------------
    goalPoss = [9,523,speed_awal_samping_kiri,10,531,speed_awal_samping_kiri,11,344,speed_awal_samping_kiri,12,678,speed_awal_samping_kiri,13,245,speed_awal_samping_kiri,14,775,speed_awal_samping_kiri,15,644,speed_awal_samping_kiri,16,380,speed_awal_samping_kiri,17,523,speed_awal_samping_kiri,18,531,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- start_kiri step 2  -----------------------
    goalPoss = [9,525,speed_awal_samping_kiri,10,540,speed_awal_samping_kiri,11,347,speed_awal_samping_kiri,12,678,speed_awal_samping_kiri,13,251,speed_awal_samping_kiri,14,775,speed_awal_samping_kiri,15,641,speed_awal_samping_kiri,16,380,speed_awal_samping_kiri,17,529,speed_awal_samping_kiri,18,539,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- start_kiri step 3  -----------------------
    goalPoss = [9,518,speed_awal_samping_kiri,10,553,speed_awal_samping_kiri,11,350,speed_awal_samping_kiri,12,683,speed_awal_samping_kiri,13,258,speed_awal_samping_kiri,14,785,speed_awal_samping_kiri,15,638,speed_awal_samping_kiri,16,375,speed_awal_samping_kiri,17,532,speed_awal_samping_kiri,18,548,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- start_kiri step 4  -----------------------
    goalPoss = [9,506,speed_awal_samping_kiri,10,567,speed_awal_samping_kiri,11,352,speed_awal_samping_kiri,12,689,speed_awal_samping_kiri,13,262,speed_awal_samping_kiri,14,798,speed_awal_samping_kiri,15,636,speed_awal_samping_kiri,16,369,speed_awal_samping_kiri,17,532,speed_awal_samping_kiri,18,557,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- start_kiri step 5  -----------------------
    goalPoss = [9,493,speed_awal_samping_kiri,10,579,speed_awal_samping_kiri,11,354,speed_awal_samping_kiri,12,693,speed_awal_samping_kiri,13,265,speed_awal_samping_kiri,14,806,speed_awal_samping_kiri,15,634,speed_awal_samping_kiri,16,365,speed_awal_samping_kiri,17,529,speed_awal_samping_kiri,18,566,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- start_kiri step 6  -----------------------
    goalPoss = [9,485,speed_awal_samping_kiri,10,587,speed_awal_samping_kiri,11,353,speed_awal_samping_kiri,12,693,speed_awal_samping_kiri,13,264,speed_awal_samping_kiri,14,806,speed_awal_samping_kiri,15,635,speed_awal_samping_kiri,16,365,speed_awal_samping_kiri,17,525,speed_awal_samping_kiri,18,573,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 0  -------------------------
    goalPoss = [9,482,speed_awal_samping_kiri,10,590,speed_awal_samping_kiri,11,352,speed_awal_samping_kiri,12,688,speed_awal_samping_kiri,13,262,speed_awal_samping_kiri,14,795,speed_awal_samping_kiri,15,636,speed_awal_samping_kiri,16,370,speed_awal_samping_kiri,17,519,speed_awal_samping_kiri,18,577,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 1  -------------------------
    goalPoss = [9,485,speed_awal_samping_kiri,10,587,speed_awal_samping_kiri,11,350,speed_awal_samping_kiri,12,678,speed_awal_samping_kiri,13,258,speed_awal_samping_kiri,14,776,speed_awal_samping_kiri,15,638,speed_awal_samping_kiri,16,380,speed_awal_samping_kiri,17,511,speed_awal_samping_kiri,18,578,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 2  -------------------------
    goalPoss = [9,489,speed_awal_samping_kiri,10,581,speed_awal_samping_kiri,11,348,speed_awal_samping_kiri,12,668,speed_awal_samping_kiri,13,254,speed_awal_samping_kiri,14,756,speed_awal_samping_kiri,15,640,speed_awal_samping_kiri,16,390,speed_awal_samping_kiri,17,503,speed_awal_samping_kiri,18,576,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 3  -------------------------
    goalPoss = [9,491,speed_awal_samping_kiri,10,573,speed_awal_samping_kiri,11,347,speed_awal_samping_kiri,12,662,speed_awal_samping_kiri,13,251,speed_awal_samping_kiri,14,743,speed_awal_samping_kiri,15,641,speed_awal_samping_kiri,16,396,speed_awal_samping_kiri,17,495,speed_awal_samping_kiri,18,572,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 4  -------------------------
    goalPoss = [9,487,speed_awal_samping_kiri,10,566,speed_awal_samping_kiri,11,346,speed_awal_samping_kiri,12,664,speed_awal_samping_kiri,13,250,speed_awal_samping_kiri,14,747,speed_awal_samping_kiri,15,642,speed_awal_samping_kiri,16,394,speed_awal_samping_kiri,17,487,speed_awal_samping_kiri,18,566,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 5  -------------------------
    goalPoss = [9,479,speed_awal_samping_kiri,10,559,speed_awal_samping_kiri,11,347,speed_awal_samping_kiri,12,670,speed_awal_samping_kiri,13,251,speed_awal_samping_kiri,14,759,speed_awal_samping_kiri,15,641,speed_awal_samping_kiri,16,388,speed_awal_samping_kiri,17,479,speed_awal_samping_kiri,18,559,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)

    #--------------------- LSL2 step 6  -------------------------
    goalPoss = [9,472,speed_awal_samping_kiri,10,551,speed_awal_samping_kiri,11,349,speed_awal_samping_kiri,12,674,speed_awal_samping_kiri,13,256,speed_awal_samping_kiri,14,767,speed_awal_samping_kiri,15,639,speed_awal_samping_kiri,16,384,speed_awal_samping_kiri,17,472,speed_awal_samping_kiri,18,551,speed_awal_samping_kiri]
    write_goalPos(goalPoss)
    msleep(41)


#======================= void LML() ======================
def LML():
    #--------------------- LML step 0  -----------------------
    global full_body
    global speed_jalan_samping
    
    goalPoss = [9,515,speed_jalan_samping,10,524,speed_jalan_samping,11,339,speed_jalan_samping,12,683,speed_jalan_samping,13,241,speed_jalan_samping,14,780,speed_jalan_samping,15,646,speed_jalan_samping,16,378,speed_jalan_samping,17,515,speed_jalan_samping,18,524,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML step 1  -----------------------
    goalPoss = [9,523,speed_jalan_samping,10,513,speed_jalan_samping,11,341,speed_jalan_samping,12,681,speed_jalan_samping,13,245,speed_jalan_samping,14,775,speed_jalan_samping,15,644,speed_jalan_samping,16,380,speed_jalan_samping,17,523,speed_jalan_samping,18,531,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML step 2  -----------------------
    goalPoss = [9,525,speed_jalan_samping,10,540,speed_jalan_samping,11,344,speed_jalan_samping,12,681,speed_jalan_samping,13,251,speed_jalan_samping,14,775,speed_jalan_samping,15,641,speed_jalan_samping,16,380,speed_jalan_samping,17,529,speed_jalan_samping,18,539,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML step 3  -----------------------
    goalPoss = [9,518,speed_jalan_samping,10,553,speed_jalan_samping,11,347,speed_jalan_samping,12,686,speed_jalan_samping,13,258,speed_jalan_samping,14,785,speed_jalan_samping,15,638,speed_jalan_samping,16,375,speed_jalan_samping,17,532,speed_jalan_samping,18,548,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML step 4  -----------------------
    goalPoss = [9,506,speed_jalan_samping,10,567,speed_jalan_samping,11,349,speed_jalan_samping,12,692,speed_jalan_samping,13,262,speed_jalan_samping,14,798,speed_jalan_samping,15,639,speed_jalan_samping,16,369,speed_jalan_samping,17,532,speed_jalan_samping,18,557,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML step 5  -----------------------
    goalPoss = [9,493,speed_jalan_samping,10,579,speed_jalan_samping,11,351,speed_jalan_samping,12,696,speed_jalan_samping,13,265,speed_jalan_samping,14,566,speed_jalan_samping,15,634,speed_jalan_samping,16,365,speed_jalan_samping,17,529,speed_jalan_samping,18,566,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML step 6  -----------------------
    goalPoss = [9,485,speed_jalan_samping,10,587,speed_jalan_samping,11,350,speed_jalan_samping,12,696,speed_jalan_samping,13,264,speed_jalan_samping,14,566,speed_jalan_samping,15,635,speed_jalan_samping,16,365,speed_jalan_samping,17,525,speed_jalan_samping,18,573,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 0  -----------------------
    goalPoss = [9,482,speed_jalan_samping,10,590,speed_jalan_samping,11,349,speed_jalan_samping,12,691,speed_jalan_samping,13,262,speed_jalan_samping,14,795,speed_jalan_samping,15,636,speed_jalan_samping,16,370,speed_jalan_samping,17,519,speed_jalan_samping,18,577,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 1  -----------------------
    goalPoss = [9,485,speed_jalan_samping,10,587,speed_jalan_samping,11,347,speed_jalan_samping,12,681,speed_jalan_samping,13,258,speed_jalan_samping,14,776,speed_jalan_samping,15,638,speed_jalan_samping,16,380,speed_jalan_samping,17,511,speed_jalan_samping,18,578,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 2  -----------------------
    goalPoss = [9,489,speed_jalan_samping,10,581,speed_jalan_samping,11,345,speed_jalan_samping,12,671,speed_jalan_samping,13,254,speed_jalan_samping,14,756,speed_jalan_samping,15,640,speed_jalan_samping,16,390,speed_jalan_samping,17,503,speed_jalan_samping,18,576,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 3  -----------------------
    goalPoss = [9,491,speed_jalan_samping,10,573,speed_jalan_samping,11,344,speed_jalan_samping,12,665,speed_jalan_samping,13,251,speed_jalan_samping,14,743,speed_jalan_samping,15,641,speed_jalan_samping,16,396,speed_jalan_samping,17,495,speed_jalan_samping,18,572,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 4  -----------------------
    goalPoss = [9,487,speed_jalan_samping,10,566,speed_jalan_samping,11,343,speed_jalan_samping,12,667,speed_jalan_samping,13,250,speed_jalan_samping,14,747,speed_jalan_samping,15,642,speed_jalan_samping,16,394,speed_jalan_samping,17,487,speed_jalan_samping,18,566,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 5  -----------------------
    goalPoss = [9,479,speed_jalan_samping,10,559,speed_jalan_samping,11,344,speed_jalan_samping,12,673,speed_jalan_samping,13,251,speed_jalan_samping,14,759,speed_jalan_samping,15,641,speed_jalan_samping,16,388,speed_jalan_samping,17,479,speed_jalan_samping,18,559,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LML2 step 6  -----------------------
    goalPoss = [9,472,speed_jalan_samping,10,551,speed_jalan_samping,11,346,speed_jalan_samping,12,677,speed_jalan_samping,13,256,speed_jalan_samping,14,767,speed_jalan_samping,15,639,speed_jalan_samping,16,384,speed_jalan_samping,17,472,speed_jalan_samping,18,551,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)


#======================= void LMR() ======================
def LMR():
    #--------------------- LMR1 step 0  -----------------------
    global full_body 
    global speed_jalan_samping

    if full_body == 1:
        goalPoss = [1,235,speed_jalan_samping,2,788,speed_jalan_samping,3,279,speed_jalan_samping,4,744,speed_jalan_samping,5,462,speed_jalan_samping,6,561,speed_jalan_samping,7,358,speed_jalan_samping,8,666,speed_jalan_samping,9,464,speed_jalan_samping,10,544,speed_jalan_samping,11,350,speed_jalan_samping,12,679,speed_jalan_samping,13,264,speed_jalan_samping,14,772,speed_jalan_samping,15,635,speed_jalan_samping,16,382,speed_jalan_samping,17,464,speed_jalan_samping,18,544,speed_jalan_samping]
    else:
        goalPoss = [7,358,speed_jalan_samping,8,666,speed_jalan_samping,9,464,speed_jalan_samping,10,544,speed_jalan_samping,11,350,speed_jalan_samping,12,679,speed_jalan_samping,13,264,speed_jalan_samping,14,772,speed_jalan_samping,15,635,speed_jalan_samping,16,382,speed_jalan_samping,17,464,speed_jalan_samping,18,544,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR1 step 1  -----------------------
    goalPoss = [9,457,speed_jalan_samping,10,536,speed_jalan_samping,11,356,speed_jalan_samping,12,680,speed_jalan_samping,13,276,speed_jalan_samping,14,773,speed_jalan_samping,15,629,speed_jalan_samping,16,381,speed_jalan_samping,17,457,speed_jalan_samping,18,536,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR1 step 2  -----------------------
    goalPoss = [9,450,speed_jalan_samping,10,532,speed_jalan_samping,11,358,speed_jalan_samping,12,679,speed_jalan_samping,13,280,speed_jalan_samping,14,772,speed_jalan_samping,15,627,speed_jalan_samping,16,382,speed_jalan_samping,17,451,speed_jalan_samping,18,528,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR1 step 3  -----------------------
    goalPoss = [9,442,speed_jalan_samping,10,534,speed_jalan_samping,11,352,speed_jalan_samping,12,678,speed_jalan_samping,13,267,speed_jalan_samping,14,769,speed_jalan_samping,15,633,speed_jalan_samping,16,383,speed_jalan_samping,17,447,speed_jalan_samping,18,520,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR1 step 4  -----------------------
    goalPoss = [9,436,speed_jalan_samping,10,538,speed_jalan_samping,11,342,speed_jalan_samping,12,676,speed_jalan_samping,13,247,speed_jalan_samping,14,765,speed_jalan_samping,15,643,speed_jalan_samping,16,385,speed_jalan_samping,17,445,speed_jalan_samping,18,512,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR1 step 5  -----------------------
    goalPoss = [9,433,speed_jalan_samping,10,541,speed_jalan_samping,11,332,speed_jalan_samping,12,674,speed_jalan_samping,13,228,speed_jalan_samping,14,761,speed_jalan_samping,15,653,speed_jalan_samping,16,387,speed_jalan_samping,17,446,speed_jalan_samping,18,504,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR1 step 6  -----------------------
    goalPoss = [9,436,speed_jalan_samping,10,538,speed_jalan_samping,11,327,speed_jalan_samping,12,673,speed_jalan_samping,13,217,speed_jalan_samping,14,759,speed_jalan_samping,15,658,speed_jalan_samping,16,388,speed_jalan_samping,17,450,speed_jalan_samping,18,498,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 0  -----------------------
    goalPoss = [9,444,speed_jalan_samping,10,530,speed_jalan_samping,11,327,speed_jalan_samping,12,672,speed_jalan_samping,13,217,speed_jalan_samping,14,758,speed_jalan_samping,15,658,speed_jalan_samping,16,389,speed_jalan_samping,17,457,speed_jalan_samping,18,494,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 1  -----------------------
    goalPoss = [9,456,speed_jalan_samping,10,517,speed_jalan_samping,11,331,speed_jalan_samping,12,674,speed_jalan_samping,13,225,speed_jalan_samping,14,761,speed_jalan_samping,15,654,speed_jalan_samping,16,387,speed_jalan_samping,17,466,speed_jalan_samping,18,491,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 2  -----------------------
    goalPoss = [9,470,speed_jalan_samping,10,505,speed_jalan_samping,11,337,speed_jalan_samping,12,676,speed_jalan_samping,13,238,speed_jalan_samping,14,765,speed_jalan_samping,15,648,speed_jalan_samping,16,385,speed_jalan_samping,17,475,speed_jalan_samping,18,491,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 3  -----------------------
    goalPoss = [9,483,speed_jalan_samping,10,498,speed_jalan_samping,11,342,speed_jalan_samping,12,679,speed_jalan_samping,13,248,speed_jalan_samping,14,772,speed_jalan_samping,15,643,speed_jalan_samping,16,382,speed_jalan_samping,17,484,speed_jalan_samping,18,494,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 4  -----------------------
    goalPoss = [9,492,speed_jalan_samping,10,500,speed_jalan_samping,11,342,speed_jalan_samping,12,682,speed_jalan_samping,13,248,speed_jalan_samping,14,778,speed_jalan_samping,15,643,speed_jalan_samping,16,379,speed_jalan_samping,17,492,speed_jalan_samping,18,500,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 5  -----------------------
    goalPoss = [9,499,speed_jalan_samping,10,508,speed_jalan_samping,11,340,speed_jalan_samping,12,684,speed_jalan_samping,13,243,speed_jalan_samping,14,782,speed_jalan_samping,15,645,speed_jalan_samping,16,377,speed_jalan_samping,17,499,speed_jalan_samping,18,508,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)

    #--------------------- LMR2 step 6  -----------------------
    goalPoss = [9,507,speed_jalan_samping,10,516,speed_jalan_samping,11,338,speed_jalan_samping,12,685,speed_jalan_samping,13,240,speed_jalan_samping,14,783,speed_jalan_samping,15,647,speed_jalan_samping,16,376,speed_jalan_samping,17,507,speed_jalan_samping,18,516,speed_jalan_samping]
    write_goalPos(goalPoss)
    msleep(24)


#===================== void tendang() ==============================
def tendang():
    #--------------------- tendang step 1 ------------------------------
    global full_body
    
    if full_body == 1:
        goalPoss = [1,235,37,2,788,37,3,279,37,4,744,37,5,462,37,6,561,37,7,358,37,8,669,37,9,441,37,10,482,37,11,374,37,12,657,37,13,298,37,14,736,69,15,636,37,16,404,37,17,442,37,18,452,21]
    else :
        goalPoss = [7,358,37,8,669,37,9,441,37,10,482,37,11,374,37,12,657,37,13,298,37,14,736,69,15,636,37,16,404,37,17,442,37,18,452,21]
    write_goalPos(goalPoss)
    msleep(3000)

    #--------------------- tendang step 2 ------------------------------
    goalPoss = [7,354,767,8,669,767,9,421,767,10,511,767,11,394,767,12,721,767,13,119,767,14,746,767,15,644,767,16,407,767,17,440,767,18,447,767]
    write_goalPos(goalPoss)
    msleep(3000)

    #--------------------- tendang step 3 ------------------------------
    goalPoss = [11,220,1023,12,626,1023,13,507,1023,14,736,1023,15,502,170,16,403,1023,17,424,1023,18,446,1023]
    write_goalPos(goalPoss)
    msleep(3000)

    #--------------------- tendang step 4 ------------------------------
    goalPoss = [9,425,69,11,272,117,12,628,255,13,301,117,14,738,255,15,556,117,16,400,255,17,432,133]
    write_goalPos(goalPoss)
    msleep(3000)

    #--------------------- tendang step  ------------------------------
    goalPoss = [11,341,20,12,682,20,13,240,20,15,647,20,17,507,20]
    write_goalPos(goalPoss)
    msleep(3000)
    walk_ready()


#====================== Duduk ===========================
def duduk():
    global full_body
    if full_body == 1:
        goalPoss = [1,182,72,2,841,72,3,294,72,4,729,72,5,490,72,6,533,72,7,353,72,8,670,72,9,508,72,17,508,72,10,515,72,18,515,72,11,268,120,12,755,120,13,71,120,14,952,120,15,753,120,16,270,120]
    else:
        goalPoss = [7,353,72,8,670,72,9,508,72,17,508,72,10,515,72,18,515,72,11,268,120,12,755,120,13,71,120,14,952,120,15,753,120,16,270,120]
    write_goalPos(goalPoss)


#==================== Jalan =============================
def jalan(kontrol):
    FML1_0()
    FML1_1()
    FML1_2()
    FML1_3()
    FML1_4()
    FML1_5()
    FML1_6()
    FML2_0()
    FML2_1()
    FML2_2()
    FML2_3()
    FML2_4()
    FML2_5()
    FML2_6()    
    kontrol_fuzzy(kontrol)
    msleep(2000)
    FMR1_0()
    FMR1_1()
    FMR1_2()
    FMR1_3()
    FMR1_4()
    FMR1_5()
    FMR1_6()
    FMR2_0()
    FMR2_1()
    FMR2_2()
    FMR2_3()
    FMR2_4()
    FMR2_5()
    FMR2_6()
    kontrol_fuzzy(kontrol)
    msleep(2000)
    


#================ MUTER KANAN ==============================
def muter_kanan():
    RTML()
    RTMR()
    kontrol_fuzzy(1)
    '''
    cek_sensor()
    if sudut0< -75:
        bangun_depan()
    else if sudut0> 75:
        bangun_belakang()
    else:
        RTML()
        RTMR()
        kontrol_fuzzy()
    '''


#===================== void MUTER KIRI =======================
def muter_kiri():
    LTML()
    LTMR()
    kontrol_fuzzy(1)
    '''  
    hitung_accelerometer();
    if (sudut0< -60) {bangun_depan();}
    else if (sudut0> 60) {bangun_belakang();}
    else
    {  LTML();
       LTMR();
       kontrol();
    }  
    '''


#================ void samping_kanan() =========================
def samping_kanan():
    start_kanan()
    msleep(500)
    RML()
    msleep(500)
    RMR()
    kontrol_fuzzy(1)
    msleep(500)
    '''
     cek();
     if (aman==0)
     {    
          start_kanan();
          delay_ms(500);
          while(aman==0)
          {
             RML();
             delay_ms(500);
             RMR();
             kontrol();
             delay_ms(500);
             cek();
             if (aman!=0)
             {
                stop_kanan();
                delay_ms(500);
             }
          }
     }
     else if (aman==1)
        {  walk_ready(); kontrol(); }
     else if (aman==2)   
          { 
            bangun_belakang();
            delay_ms(600);
          }            
     else if (aman==3)
          {  
            bangun_depan();
            delay_ms(600);
          } 
'''


#======================= void samping_kiri() =====================
def samping_kiri():
    start_kiri()
    msleep(500)
    LMR()
    msleep(500)
    LML()
    kontrol_fuzzy(1)
    msleep(500)
    '''
     cek();
     if (aman==0)
     {    
          start_kiri();
          delay_ms(500);
          while(aman==0)
          {
             LMR();
             delay_ms(500);
             LML();
             kontrol();
             delay_ms(500);
             cek();
             if (aman!=0)
             {
                stop_kiri();
                delay_ms(500);
             }
          }
     }
     else if (aman==1)
        {  walk_ready(); kontrol(); }
     else if (aman==2)   
          { 
            bangun_belakang();
            delay_ms(600);
          }            
     else if (aman==3)
          {  
            bangun_depan();
            delay_ms(600);
          } 
    '''

def walk_ready(): 
    global full_body
    global walkready
    if full_body == 1:
        if verMX == 1:
            #goalPoss = [1,350,72,2,673,72,3,250,72,4,773,72,5,300,72,6,723,72,7,358,72,8,666,72,9,507,72,17,507,72,10,516,72,18,516,72,11,346,72,12,677,72,13,240,220,14,783,220,15,647,120,16,376,120]
            goalPoss = [1,235,72,2,788,72,3,279,72,4,744,72,5,462,72,6,561,72,7,358,72,8,666,72,9,507,72,17,507,72,10,516,72,18,516,72,11,toMX(346),toMXspeed(72),12,toMX(677),toMXspeed(72),13,toMX(240),toMXspeed(220),14,toMX(783),toMXspeed(220),15,toMX(647),toMXspeed(120),16,toMX(376),toMXspeed(120)]
        else:
            goalPoss = [1,235,72,2,788,72,3,279,72,4,744,72,5,462,72,6,561,72,7,358,72,8,666,72,9,507,72,17,507,72,10,516,72,18,516,72,11,346,72,12,677,72,13,240,220,14,783,220,15,647,120,16,376,120]
    else :
        goalPoss = [7,358,72,8,666,72,9,507,72,17,507,72,10,516,72,18,516,72,11,346,72,12,677,72,13,240,220,14,783,220,15,647,120,16,376,120]
    write_goalPos(goalPoss)
    walkready = 1
    msleep(800)


# ====================== PROGRAM UTAMA ==============================
# =================================== PROGRAM UTAMA ===================================
# =================================== SetupBipedRobot2.m ==============================
# SetupBipedRobot2.m
# Biped walking robot structure data: See Figure 2.19, Figure 2.20
# For the definition of each field, see "Table 2.1 Link information"
ToDeg = 180/math.pi
ToRad = math.pi/180
# range maksimum sudut servo AX12 atau MX28 yang diperbolehkan ! 
# PENTING !!
maxServo = [1,0,1023,2,0,1023,7,225,570,8,454,799,9,364,660,10,364,660,11,976,2320,12,1776,3120,13,646,2048,14,2048,3450,15,1833,3164,16,932,2263,17,383,665,18,383,665]

# set nilai b, WAJIB!
L1 = 3.85    #3.6
L2 = 7.519
L3 = 7.432
L4 = 3.233
# setpoint servo agar lurus menghadap depan
spka = 358
spki = 666
# mulai update parameter b semua link dan p(BODY)
uLINK[1].R = np.eye(3, dtype = float)        # BODY
set_b(L1,L2,L3,L4)                           # update parameter b
FindMother(1)

# inisialisasi komunikasi serial raspi-arduino
ser2 = serial.Serial('/dev/ttyUSB1', baudrate = 57600, timeout = 1)

# inisialisasi servo AX12
dxl_init()

# enable torsi
if full_body == 1:
    range_servo = 18
else :
    range_servo = 12

for i in range(range_servo):
    if full_body == 1:
        dxl_enableTorque(i+1)
        print(i+1)
    else :
        dxl_enableTorque(i+7)
        print(i+7)

walk_ready()
msleep(2000)
# inisialisasi kalibrasi dan baca sensor
kalibrasi = 2
kontrol = 1

kontrol_fuzzy(kalibrasi)
while(1):
    #jalan(kontrol)
    kontrol_fuzzy(kontrol)