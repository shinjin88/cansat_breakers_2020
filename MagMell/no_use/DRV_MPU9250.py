import smbus
import time
import threading
import serial

class MPU9250:
    #i2c0→SDA:GPIO0 SCL:GPIO1
    #i2c1→SDA:GPIO2 SCL:GPIO3

    ADDR_MPU6050 = 0x68#accel&gyro

    PWR_MGMT_1 = 0x6b
    H_RESET = 0x80
    CLK_SEL_Internal_20MHZ = 0x00
    CLK_SEL_Auto_SELECT = 0x01

    PWR_MGMT_2 = 0x6c
    INT_PIN_CFG = 0x37#Enable AK8693

    MPU_CONFIG = 0x1a
    #DLPF_CFG_SEL_BW_8800HZ = 0x00
    #DLPF_CFG_SEL_BW_3600HZ = 0x01
    DLPF_CFG_SEL_BW_250HZ = 0x03
    #DLPF_CFG_SEL_BW_184HZ = 0x07

    SMPLRT_DIV = 0x19

    ACCEL_CONFIG = 0x1c
    ACCEL_FS_SEL_2G = 0x00
    ACCEL_FS_SEL_4G = 0x08
    ACCEL_FS_SEL_8G = 0x10
    ACCEL_FS_SEL_16G = 0x18
    AccRange = 2.0#default
    ACCEL_XOUT_H = 0x3b
    ACCEL_YOUT_H = 0x3d
    ACCEL_ZOUT_H = 0x3f
    AccCoefficient = AccRange/float(0x8000)
    offsetAccelX = 0
    offsetAccelY = 0
    offsetAccelZ = 0

    ACCEL_CONFIG2 = 0x1d
    A_DLPF_CFG_BW_218HZ = 0x03

    GYRO_CONFIG = 0x1b
    GYRO_FS_SEL_250DPS = 0x00
    GYRO_FS_SEL_500DPS = 0x08
    GYRO_FS_SEL_1000DPS = 0x10
    GYRO_FS_SEL_2000DPS = 0x18
    GyroRange = 250#default
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47
    GyroCoefficient = GyroRange/float(0x8000)
    offsetGyroX = 0
    offsetGyroY = 0
    offsetGyroZ = 0

    TEMP_OUT_H = 0x41

    AK8693_FLAG = False

    ADDR_AK8963 = 0x0c#magnet
    CNTL1 = 0x0a
    CNTL1_MODE_SINGLE = 0x01
    CNTL1_MODE_SEL_8HZ = 0x02
    CNTL1_MODE_SEL_100HZ = 0x06
    CNTL2 = 0x0b#reset
    ST1 = 0x02
    ST2 = 0x09
    HX_L = 0x03
    HY_L = 0x05
    HZ_L = 0x07
    MagBit = 14#default
    magmode = CNTL1_MODE_SEL_8HZ#default
    MagCoefficient16 = 4912/32760
    MagCoefficient14 = 4912/8190
    MagOffsetX = 0 
    MagOffsetY = 0
    MagOffsetZ = 0
    #-9.914588082849438,2.308549153168272,-9.97619164380884,43.063643134874134

    ACCEL_XYZ = [0,0,0]
    GYRO_RPY = [0,0,0]
    TIME_STAMP = 0

    serFlag = False

    TransmitIntervar = 0.01
    

    def __init__(self,channel=1,AccRange=2,GyroRange=250,calibAcc=False,calibGyro=False):
        self.channel = channel
        self.bus = smbus.SMBus(self.channel)
        self.resetRegister()
        self.configMPU6050(AccRange=2,GyroRange=250)
        self.configAK8693()

        if calibAcc == True:
            self.calibAccel()
        if calibGyro == True:
            self.calibGyro()

    def resetRegister(self):
        if self.AK8693_FLAG == True:
            self.bus.write_byte_data(self.ADDR_AK8963,self.CNTL2,0x01)
            time.sleep(0.1)
            self.AK8693_FLAG = False

        self.bus.write_byte_data(self.ADDR_MPU6050,self.PWR_MGMT_1,self.H_RESET)
        time.sleep(0.1)

    def configMPU6050(self,AccRange=2,GyroRange=250):
        if AccRange == 2:
            accvar = self.ACCEL_FS_SEL_2G
            self.AccRange = 2.0
        elif AccRange == 4:
            accvar = self.ACCEL_FS_SEL_4G
            self.AccRange = 4.0
        elif AccRange == 8:
            accvar = self.ACCEL_FS_SEL_8G
            self.AccRange = 8.0
        elif AccRange == 16:
            accvar = self.ACCEL_FS_SEL_16G
            self.AccRange = 16.0
        else:
            raise Exception('Invalid Accel-range')

        self.AccCoefficient = self.AccRange/float(0x8000)
        self.offsetAccelX = 0
        self.offsetAccelY = 0
        self.offsetAccelZ = 0        

        if GyroRange == 250:
            gyrovar = self.GYRO_FS_SEL_250DPS
            self.GyroRange = 250.0
        elif GyroRange == 500:
            gyrovar = self.GYRO_FS_SEL_500DPS
            self.GyroRange = 500.0
        elif GyroRange == 1000:
            gyrovar = self.GYRO_FS_SEL_1000DPS
            self.GyroRange = 1000.0
        elif GyroRange == 2000:
            gyrovar = self.GYRO_FS_SEL_2000DPS
            self.GyroRange = 16.0
        else:
            raise Exception('Invalid Gyro-range')

        self.GyroCoefficient = self.GyroRange/float(0x8000)
        self.offsetGyroX = 0
        self.offsetGyroY = 0
        self.offsetGyroZ = 0

        self.bus.write_byte_data(self.ADDR_MPU6050,self.PWR_MGMT_1,0x00)#wakeup
        time.sleep(0.1)
        self.bus.write_byte_data(self.ADDR_MPU6050,self.PWR_MGMT_1,self.CLK_SEL_Auto_SELECT)#select clock source
        time.sleep(0.1)
        self.bus.write_byte_data(self.ADDR_MPU6050,self.MPU_CONFIG,self.DLPF_CFG_SEL_BW_250HZ)#enable DLPF
        time.sleep(0.1)
        self.bus.write_byte_data(self.ADDR_MPU6050,self.SMPLRT_DIV,0x04)
        time.sleep(0.1)
        self.bus.write_byte_data(self.ADDR_MPU6050,self.ACCEL_CONFIG,accvar)
        time.sleep(0.1)
        self.bus.write_byte_data(self.ADDR_MPU6050,self.ACCEL_CONFIG2,self.A_DLPF_CFG_BW_218HZ)
        time.sleep(0.1)
        self.bus.write_byte_data(self.ADDR_MPU6050,self.GYRO_CONFIG,gyrovar)
        time.sleep(0.1)
        
        
    def configAK8693(self,mode='8HZ',bit='14bit',calib=False):
        if self.AK8693_FLAG == False:
            self.bus.write_byte_data(self.ADDR_MPU6050,self.INT_PIN_CFG,0x02)
            time.sleep(0.1)
            self.AK8693_FLAG = True
        
        if mode == '8HZ':
            mode_ = self.CNTL1_MODE_SEL_8HZ
            self.magmode = mode_
        elif mode == '100HZ':
            mode_ = self.CNTL1_MODE_SEL_100HZ
            self.magmode = mode_
        elif mode == 'SINGLE':
            mode_ = self.CNTL1_MODE_SINGLE
            self.magmode = mode_
        else:
            raise Exception('Invalid Interval Config')

        if bit == '14bit':
            mode_ = mode_ | 0x00
            self.MagBit = 14
        elif bit == '16bit':
            mode_ = mode_ | 0x10
            self.MagBit = 16

        self.bus.write_byte_data(self.ADDR_AK8963,self.CNTL1,mode_)
        time.sleep(0.1)

        if calib == True:
            self.MagOffsetX = -9.914588082849438
            self.MagOffsetY = 2.308549153168272
            self.MagOffsetZ = -9.97619164380884


    @classmethod
    def us2s(cls,unsigned_num):
        if unsigned_num & (0x01 << 15):
            return -1*((unsigned_num^0xffff)+1)

        return unsigned_num

    def readAccel(self):
        var = self.bus.read_i2c_block_data(self.ADDR_MPU6050,self.ACCEL_XOUT_H,6)
        rawX = self.us2s(var[0]<<8|var[1])*self.AccCoefficient-self.offsetAccelX
        rawY = self.us2s(var[2]<<8|var[3])*self.AccCoefficient-self.offsetAccelY
        rawZ = self.us2s(var[4]<<8|var[5])*self.AccCoefficient-self.offsetAccelZ

        return [rawX,rawY,rawZ]

    def calibAccel(self,count=1000):
        print('calibration:ACCEL Started.')
        sum_ = [0,0,0]
        for i in range(count):
            data = self.readAccel()
            sum_[0] += data[0]
            sum_[1] += data[1]
            sum_[2] += (data[2]-1)

        self.offsetAccelX = sum_[0]/count
        self.offsetAccelY = sum_[1]/count
        self.offsetAccelZ = sum_[2]/count
        print('calibration:ACCEL Completed.')
        print('offsetX:'+str(self.offsetAccelX))
        print('offsetY:'+str(self.offsetAccelY))
        print('offsetZ:'+str(self.offsetAccelZ))

    def readGyro(self):
        var = self.bus.read_i2c_block_data(self.ADDR_MPU6050,self.GYRO_XOUT_H,6)
        rawX = self.us2s(var[0]<<8|var[1])*self.GyroCoefficient-self.offsetGyroX
        rawY = self.us2s(var[2]<<8|var[3])*self.GyroCoefficient-self.offsetGyroY
        rawZ = self.us2s(var[4]<<8|var[5])*self.GyroCoefficient-self.offsetGyroZ
        
        return [rawX,rawY,rawZ]

    def calibGyro(self,count=1000):
        print('calibration:GYRO Started.')
        sum_ = [0,0,0]
        for i in range(count):
            data = self.readGyro()
            sum_[0] += data[0]
            sum_[1] += data[1]
            sum_[2] += data[2]

        self.offsetGyroX = sum_[0]/count
        self.offsetGyroY = sum_[1]/count
        self.offsetGyroZ = sum_[2]/count
        print('calibration:GYRO Completed.')
        print('offsetX:'+str(self.offsetGyroX))
        print('offsetY:'+str(self.offsetGyroY))
        print('offsetZ:'+str(self.offsetGyroZ))

    def read6Axis(self):
        #self.TIME_STAMP = time.time()
        self.ACCEL_XYZ = self.readAccel()
        self.GYRO_RPY = self.readGyro()
        time.sleep(0.001)

    def launch6Axis(self):
        self.imuthread = threading.Thread(target=self.read6Axis,args = ())
        self.imuthread.daemon = True#デーモン化　
        self.imuthread.start() 

    def readMag(self):
        if self.magmode == (self.CNTL1_MODE_SEL_8HZ or self.CNTL1_MODE_SEL_100HZ):
            stat = self.bus.read_i2c_block_data(self.ADDR_AK8963,self.ST1,1)
            if (stat[0]&0x02) == 0x02:
                stat = self.bus.read_i2c_block_data(self.ADDR_AK8963,self.ST2,1)

        stat = self.bus.read_i2c_block_data(self.ADDR_AK8963,self.ST1,1)
        while ((stat[0]&0x01) != 0x01):
            stat = self.bus.read_i2c_block_data(self.ADDR_AK8963,self.ST1,1)

        var = self.bus.read_i2c_block_data(self.ADDR_AK8963,self.HX_L,7)
        rawX = self.us2s(var[1]<<8|var[0])
        rawY = self.us2s(var[3]<<8|var[2])
        rawZ = self.us2s(var[5]<<8|var[4])
        
        if (var[6]&0x08) == 0x08:
            raise Exception('Error:Magnet Sensor is Over-Flowed!')

        if self.MagBit == 16:
            rawX = rawX*self.MagCoefficient16
            rawY = rawY*self.MagCoefficient16
            rawZ = rawZ*self.MagCoefficient16
        else:
            rawX = rawX*self.MagCoefficient14
            rawY = rawY*self.MagCoefficient14
            rawZ = rawZ*self.MagCoefficient14

        if self.magmode == self.CNTL1_MODE_SINGLE:
            mode_ = self.CNTL1_MODE_SINGLE
            if self.MagBit == 14:
                mode_ = mode_ | 0x00
            elif self.MagBit == 16:
                mode_ = mode_ | 0x10

            self.bus.write_i2c_block_data(self.ADDR_AK8963,self.CNTL1,[mode_])
            time.sleep(0.01)

        return [rawX+self.MagOffsetX,rawY+self.MagOffsetY,rawZ+self.MagOffsetZ]

    def calibMagnet(self,name = 'Magnet_Calibration.csv',gain=0.00001,count=1000):
        with open(name,mode='w') as file:
            pass
        #gain = 0.0001
        bias = [0,0,0,1]#[x,y,z,R]
        print('センサを8の字に回してください')
        print('calibration:MAGNET started.')
        for i in range(count):
            print('\nCOUNT:')
            print(i)
            Mgn = self.readMag()
            time.sleep(0.1)
            dx = Mgn[0] - bias[0]
            dy = Mgn[1] - bias[1]
            dz = Mgn[2] - bias[2]
            e = dx**2+dy**2+dz**2-bias[3]**2
            bias[0] = bias[0]+4*gain*e*dx
            bias[1] = bias[1]+4*gain*e*dy
            bias[2] = bias[2]+4*gain*e*dz
            bias[3] = bias[3]+4*gain*e*bias[3]
            print(e)
            with open(name,mode='a') as file:
                file.write('\n'+str(bias[0])+','+str(bias[1])+','+str(bias[2])+','+str(bias[3]))
            
    def catchDropMk1(self,var=0.2):
        print('Waiting...')
        while True:
            Acc = self.readAccel()
            Abs = (Acc[0]**2+Acc[1]**2+Acc[2]**2)**(1/2)
            time.sleep(0.001) 
            if Abs < var:
                print('Dropped!')
                return

    def catchDropMk2(self,var1=0.2,var2=0.8,testmode=False):
        if testmode == True:
            time.sleep(5)
            return True
        s = serial.Serial(ser='dev/serial0',baudrate=4800,timeout=10)
        self.WaitFlag = 0
        while True:
            Acc = self.readAccel()
            Abs = (Acc[0]**2+Acc[1]**2+Acc[2]**2)**(1/2)
            if Abs < var1:
                #print('Dropped?')
                if self.serFlag == True:
                    s.write(('Dropped? Judging...\n').encode('utf-8'))
                self.WaitFlag = 1
                starttime = time.time()
                while True:
                    Acc = self.readAccel()
                    Abs = (Acc[0]**2+Acc[1]**2+Acc[2]**2)**(1/2)
                    if Abs > var2:
                        self.WaitFlag = 2
                        if self.serFlag == True:
                            s.write(('Dropped.\n').encode('utf-8'))
                            s.close()
                        return True
                    if (time.time()-starttime)>2:
                        self.WaitFlag = 0
                        if self.serFlag == True:
                            s.write(('Not Dropped. Try Again\n').encode('utf-8'))
                        break

