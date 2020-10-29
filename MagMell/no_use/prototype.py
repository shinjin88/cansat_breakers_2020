from LibIMU import LibIMU
from DRV_GPIO import MotorControl,Servo,Drill,LED
from DRV_GPS_Serial import Gps_Serial
from DRV_Camera import Camera
import time

#Goal Ref
GOAL_LAT = 0
GOAL_LNG = 0

#setting
CHANGE_MODE = False
TEST_MODE = False
DRILL_ENABLE = False

#GPIO Number
#Mortor
MR1 = 17
MR2 = 27
ML1 = 22
ML2 = 10
#Servo
SERVO = 18
#Mortor(drill_Elevator)
MD1 = 9
MD2 = 11
#Motor(drill)
MD3 = 5
MD4 = 6
#Encoder(drill)
ED1 = 23
ED2 = 24
#LED
LED1 = 20
LED2 = 21

#Etc Setting
#Turn Coefficient
TURN_Coefficient = 1.2
#Range for modechange
Range = 8
#Range for Goal
Gdistance = 3
#Moving Time(FirstTime)
MoveTime = 8
#GPS Distance Coefficient
GPS_D_Coefficient = 0.7
#Drill
MaxLength = 100
Gear_Radius = 10
GetTime = 8
#permissible degrees on moving forward
maxdeg = 8
#speed
speed = 1.25





def RunBack(lat,lng,test=False):

    LatLngG = {
    'lat':lat,
    'lng':lng
    }

    #初期化
    DRV_GPS_Ser = Gps_Serial()
    DRV_GPS_Ser.launchGps()
    DRV_GPS_Ser.transmitData('RDY:GPS&Serial. GPIONum:14(TX,Serial),15(RX,GPS)')
    DRV_GPS_Ser.transmitData('Can You Hear Me?')
    DRV_LED = LED(LED1,LED2)
    DRV_LED.startBlink(alt=False)
    DRV_GPS_Ser.transmitData('DRY:LED. GPIONum:'+str(LED1)+','+str(LED2))
    DRV_Mortor = MotorControl(MR1,MR2,ML1,ML2)
    DRV_GPS_Ser.transmitData('RDY:Mortor. GPIONum: MR1:'+str(MR1)+',MR2:'+str(MR2)+',ML1:'+str(ML1)+',ML2:'+str(ML2))
    DRV_Servo = Servo(SERVO)
    DRV_GPS_Ser.transmitData('RDY:Servo. GPIONum:'+str(SERVO))
    DRV_Camera = Camera()
    DRV_GPS_Ser.transmitData('RDY:Camera.')
    DRV_GPS_Ser.transmitData('CAUTION!:calibrating IMU. DONT TOUCH ME.')
    IMU = LibIMU()
    DRV_GPS_Ser.transmitData('RDY:IMU. GPIONum:2(SDA),3(SCL)')
    DRV_GPS_Ser.transmitData('Initialized.')
    DRV_GPS_Ser.transmitData('Mode:GPS')

    #投下待機
    DRV_GPS_Ser.transmitData('Waiting...')
    DRV_GPS_Ser.stopSerial()
    DRV_LED.changeBlinkPattern(pat='alternate')
    IMU.DRV.catchDropMk2(testmode=test)
    DRV_LED.changeBlinkPattern(pat='no_alternate')
    DRV_GPS_Ser.reconnectSerial()
    DRV_GPS_Ser.transmitData('Falling!')

    #10秒待つ
    DRV_GPS_Ser.transmitData('Waiting for 10 sec...')
    time.sleep(10)

    #パラシュートをパージ
    DRV_GPS_Ser.transmitData('Purging...')
    DRV_Servo.swing()
    DRV_Servo.terminate()

    #カバーから出る
    DRV_Mortor.moveForward(1)

    #ドリル
    if DRILL_ENABLE == True:
        Driller()

    #GPS受信(初回)
    DRV_LED.changeBlinkPattern(pat='alternate')
    LatLng0 = DRV_GPS_Ser.readGPSData()
    DRV_GPS_Ser.transmitData('1st GPS Data Recieved.')
    DRV_GPS_Ser.transmitData('Latitude:'+str(LatLng0['lat'])+',Longitude:'+str(LatLng0['lng']))
    Distance2 = DRV_GPS_Ser.getDistance2(LatLng0,LatLngG)

    #進行方向を電子コンパスより取得し、ゴールの方向へ旋回
    myCourse = Distance2['distance'] - IMU.getYaw_single()
    if myCourse < -180:
        myCourse += 360
    elif myCourse > 180:
        myCourse -= 360 
    if myCourse < 0 :
        DRV_Mortor.turnLeft_St()
        IMU.launch6Axis('yaw')
        while True:
            if IMU.EA_YAW < myCourse:
                break
    elif myCourse > 0:
        DRV_Mortor.turnRight_St()
        IMU.launch6Axis('yaw')
        while True:
            if IMU.EA_YAW < myCourse:
                break
    

    #ループスタート
    while True:
        if Distance2['distance'] < Range:#8m未満ならば
            if Distance2['distance'] > Gdistance:#3m以上ならば
                if CHANGE_MODE == False:
                    DRV_GPS_Ser.transmitData('Mode:Image')
                CHANGE_MODE = True#終末誘導モードへ
            else:#3m以内ならばゴール
                DRV_GPS_Ser.transmitData('GOAL. Distance:'+str(Distance2['distance'])+'m')
                while True:
                    pass
        else:#そうでなければ
            if CHANGE_MODE == True:
                DRV_GPS_Ser.transmitData('Mode:GPS')
            CHANGE_MODE = False#通常モードへ

        if CHANGE_MODE == False:#通常モードならば

            #しばらく前進
            DRV_GPS_Ser.startMark()
            IMU.launch6Axis(mode='pos_yaw')
            moveStart = time.time()
            DRV_Mortor.moveForward_St()
            while (time.time() - moveStart) < MoveTime:
                if IMU.EA_YAW > maxdeg:
                    lossTime = time.time()
                    DRV_Mortor.turnLeft_St()
                    while True:
                        if IMU.EA_YAW < maxdeg:
                            DRV_Mortor.moveForward_St()
                            MoveTime += time.time() - lossTime
                            break
                elif IMU.EA_YAW < -(maxdeg):
                    lossTime = time.time()
                    DRV_Mortor.turnLeft_St()
                    while True:
                        if IMU.EA_YAW > -(maxdeg):
                            DRV_Mortor.moveForward_St()
                            MoveTime += time.time() - lossTime
                            break
            DRV_Mortor.stopMotor()

            DRV_GPS_Ser.readGPSData()
            DRV_GPS_Ser.stopMark()
            IMU.killThread()

            #GPSデータから確からしい進路を推測し、現在の座標を算出する
            LatLng1 = DRV_GPS_Ser.getPoint(DRV_GPS_Ser.markstop)
            DRV_GPS_Ser.transmitData('GPS Data Recieved.')
            DRV_GPS_Ser.transmitData('Latitude:'+str(LatLng1['lat'])+',Longitude:'+str(LatLng1['lng']))

            if (IMU.ACC_DX<1):
                DRV_GPS_Ser.transmitData('Stacked!. Escaping...')
                IMU.launch6Axis('yaw')
                DRV_Mortor.escapeStack()
                IMU.killThread()
                turn_reset = -IMU.EA_YAW
                DRV_Mortor.moveForward(2)
                IMU.launch6Axis('yaw')
                if turn_reset < 0:
                    DRV_Mortor.turnLeft_St()
                    while True:
                        if IMU.EA_YAW*TURN_Coefficient < turn_reset:
                            DRV_Mortor.stopMotor()
                            DRV_GPS_Ser.transmitData('Turned(deg):'+str(IMU.EA_YAW)+'from Gyro-Sensor')
                            break
                else:
                    DRV_Mortor.turnRight_St()
                    while True:
                        if IMU.EA_YAW*TURN_Coefficient > turn_reset:
                            DRV_Mortor.stopMotor()
                            DRV_GPS_Ser.transmitData('Turned(deg):'+str(IMU.EA_YAW)+'from Gyro-Sensor')
                            break
                IMU.killThread()
            
            #距離と方位角を計算する
            Distance1 = DRV_GPS_Ser.getDistance2(LatLng0,LatLng1)#前回から現在へ
            if Distance1 > 3:#3m以上移動していれば方位にはGPSのデータを使う
                pass
            else:#そうでなければ電子コンパスのデータを使う
                Distance1['courseS2G'] = IMU.getYaw_single()
            distance2 = DRV_GPS_Ser.getDistance2(LatLng1,LatLngG)#現在からゴールへ
            if distance2['distance'] < Range:
                LatLng0 = LatLng1
                continue
            if distance2['distance'] < Distance2['distance']:
                Distance2 = distance2
                MoveTime = (Distance2/speed)*0.8
                DRV_GPS_Ser.transmitData('Distance to Goal:'+str(Distance2['distance']))
                DRV_GPS_Ser.transmitData('Keeping this Cource...')
                continue
            Distance2 = distance2
            
            Cource_Angle = Distance2['courseS2G'] - Distance1['courseS2G']#旋回すべき角度を算出する
            if Cource_Angle >= 180:
                Cource_Angle -= 360
            elif Cource_Angle <= -180:
                Cource_Angle += 360
            #MoveTime = Distance2/((Distance1['distance']*GPS_D_Coefficient + IMU.ACC_DX*(1-GPS_D_Coefficient))/MoveTime)#前進すべき時間の算出
            MoveTime = (Distance2['distance']/speed)*0.5

            DRV_GPS_Ser.transmitData('Distance to Goal:'+str(Distance2['distance']))
            DRV_GPS_Ser.transmitData('Course(Angle) to Goal:'+str(Cource_Angle))

            #所定の角度になるまで旋回
            IMU.launch6Axis(mode='yaw')
            if Cource_Angle < 0:
                DRV_Mortor.turnLeft_St()
                while True:
                    if IMU.EA_YAW*TURN_Coefficient < Cource_Angle:
                        DRV_Mortor.stopMotor()
                        DRV_GPS_Ser.transmitData('Turned(deg):'+str(IMU.EA_YAW)+'from Gyro-Sensor')
                        break
            else:
                DRV_Mortor.turnRight_St()
                while True:
                    if IMU.EA_YAW*TURN_Coefficient > Cource_Angle:
                        DRV_Mortor.stopMotor()
                        DRV_GPS_Ser.transmitData('Turned(deg):'+str(IMU.EA_YAW)+'from Gyro-Sensor')
                        break
            IMU.killThread()

            #座標を更新
            LatLng0 = LatLng1

        elif CHANGE_MODE == True:
            while True:
                DRV_GPS_Ser.transmitData('Taking a Picture...')
                DRV_Camera.getImage()
                DRV_GPS_Ser.transmitData('Detecting...')
                var = DRV_Camera.detectTarget()
                if var == False:
                    DRV_Mortor.turnRight(0.3)
                    continue
                else:
                    DRV_GPS_Ser.transmitData('Target Acquired.')
                    if DRV_Camera.judge():
                        DRV_GPS_Ser.transmitData('Target is Front.')
                        DRV_Mortor.moveForward(1)
                        Distance2 = DRV_GPS_Ser.readGPSData()
                        break
                    else:
                        if var < 512:
                            DRV_Mortor.turnLeft(0.2)
                            continue
                        else:
                            DRV_Mortor.turnRight(0.2)
                            continue
                
def Driller():
    DRV_Drill = Drill(MD1,MD2,MD3,MD4,ED1,ED2,Gear_Radius,MaxLength)
    DRV_Drill.downDrill()
    DRV_Drill.spinDrill()
    DRV_Drill.downDrill(10,0)
    time.sleep(GetTime)
    DRV_Drill.stopDrill()
    DRV_Drill.upDrill(speed=50)

def main():
    pass

if __name__ == "__main__":
    RunBack(GOAL_LAT,GOAL_LNG)          
