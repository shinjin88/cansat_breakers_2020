from LibIMU import LibIMU
from DRV_GPIO import MotorControl,Servo,Drill,LED
from DRV_GPS_Serial import Gps_Serial
from DRV_Camera import Camera
import numpy
import time
import os
import json
import  csv

class RunBack:
	
	print_to_console = False

	def __init__(self,GOAL_LAT,GOAL_LNG,GOAL_Distance,console=False):

		file = open('./config.json','r')
		conf = json.load(file)
		#GPIO Number
		#Mortor
		self.MR1 = conf['config']['pin']['Mortor']['MR1']
		self.MR2 = conf['config']['pin']['Mortor']['MR2']
		self.MR_VREF = conf['config']['pin']['Mortor']['MR_VREF']
		self.ML1 = conf['config']['pin']['Mortor']['ML1']
		self.ML2 = conf['config']['pin']['Mortor']['ML2']
		self.ML_VREF = conf['config']['pin']['Mortor']['ML_VREF']
		#Servo
		self.SERVO = conf['config']['pin']['Servo']
		#Drill(Spec)
		self.drill_leng = conf['config']['control']['spec']['drill_updown_length']
		self.drill_gear_rad = conf['config']['control']['spec']['drill_gear_rad']
		#Mortor(drill_Elevator)
		self.MD1 = conf['config']['pin']['Drill']['MD1']
		self.MD2 = conf['config']['pin']['Drill']['MD2']
		self.MDVREF = conf['config']['pin']['Drill']['MDVREF']
		#Motor(drill)
		self.MD3 = conf['config']['pin']['Drill']['MD3']
		self.MD4 = conf['config']['pin']['Drill']['MD4']
		#Encoder(drill)
		self.ED1 = conf['config']['pin']['Drill']['ED1']
		self.ED2 = conf['config']['pin']['Drill']['ED2']
		#LED
		self.LED1 = conf['config']['pin']['LED']['LED1']
		self.LED2 = conf['config']['pin']['LED']['LED2']
		#etc
		self.TURN_Coefficient = conf['config']['control']['spec']['turn']
		self.speed = conf['config']['control']['spec']['speed']
		self.Kp = conf['config']['control']['spec']['Kp']
		self.Ti = conf['conf']['control']['spec']['Ti']
		self.Td = conf['conf']['control']['spec']['Td']
		self.width = conf['config']['control']['spec']['width']
		self.diameter = conf['config']['control']['spec']['diameter']
		self.tagRPS = self.speed/(self.diameter*numpy.pi)

		self.roll_coe1 = self.width/(2*self.diameter)
		self.roll_coe2 = 2*numpy.pi

		file.close()

		if console == True:
			self.print_to_console = True

		self.LatLngG = {
			'lat':GOAL_LAT,
			'lng':GOAL_LNG
		}
		self.LatLngB = {
			'lat':0,
			'lng':0
		}
		self.LatLngN = {
			'lat':0,
			'lng':0
		}

		#初期化
		self.DRV_GPS_Ser = Gps_Serial()
		if console == True:
			self.transmit('Console Debug Mode Enabled.')
		self.DRV_GPS_Ser.launchGps()
		self.transmit('RDY:GPS&Serial. GPIONum:14(TX,Serial),15(RX,GPS)')
		self.transmit()
		self.DRV_LED = LED(self.LED1,self.LED2)
		self.DRV_LED.startBlink(alt=False,interval=5.0)
		self.transmit('DRY:LED. GPIONum:'+str(self.LED1)+','+str(self.LED2))
		self.DRV_Mortor = MotorControl(self.MR1,self.MR2,self.MR_VREF,self.ML1,self.ML2,self.ML_VREF)
		self.transmit('RDY:Mortor. GPIONum: MR1:'+str(self.MR1)+',MR2:'+str(self.MR2)+',ML1:'+str(self.ML1)+',ML2:'+str(self.ML2))
		self.DRV_Servo = Servo(self.SERVO)
		self.transmit('RDY:Servo. GPIONum:'+str(self.SERVO))
		self.DRV_Camera = Camera()
		self.transmit('RDY:Camera.')
		self.transmit('CAUTION!:calibrating IMU. DONT TOUCH ME.')
		self.IMU = LibIMU()
		self.transmit('RDY:IMU. GPIONum:2(SDA),3(SCL)')
		self.DRV_Drill = Drill(self.MD1,self.MD2,self.MDVREF,self.MD3,self.MD4,self.ED1,self.ED2,self.drill_gear_rad,self.drill_leng)
		self.transmit('RDY:Drill.')
		self.transmit('Mode:GPS')
		self.transmit('Initialized.')
		self.transmit(r" ____                 _ ")                
		self.transmit(r"| __ ) _ __ ___  __ _| | _____ _ __ ___     _____     ____")
		self.transmit(r"|  _ \| '__/ _ \/ _` | |/ / _ \ '__/ __|   /\/\/\/\  |  o | ")
		self.transmit(r"| |_) | | |  __/ (_| |   <  __/ |  \__ \  |\/\/\/\/|/ ___\| ")
		self.transmit(r"|____/|_|  \___|\__,_|_|\_\___|_|  |___/  |/\/\/\/\/_/   ")
		self.transmit(r"                                          |_|_| |_|_|")
		self.transmit(r" ____            _                                   ___   ___")
		self.transmit(r"/ ___| _   _ ___| |_ ___ _ __ ___    _ __ _____   __/ _ \ ( _ )")
		self.transmit(r"\___ \| | | / __| __/ _ \ '_ ` _ \  | '__/ _ \ \ / / | | |/ _ \/")
		self.transmit(r" ___) | |_| \__ \ ||  __/ | | | | | | | |  __/\ V /| |_| | (_) |")
		self.transmit(r"|____/ \__, |___/\__\___|_| |_| |_| |_|  \___| \_(_)\___(_)___/")
		self.transmit(r"       |___/                                                   ")

	def changeLEDPattern(self,mode='alternate'):
		self.DRV_LED.changeBlinkPattern(pat=mode)

	def transmit(self,txt='Can you hear me?'):
		if self.print_to_console == True:
			print(str(txt))
		else:
			self.DRV_GPS_Ser.transmitData(txt)

	def waitDrop(self):
		#投下待機
		self.transmit('Waiting...')
		self.DRV_GPS_Ser.stopSerial()
		self.DRV_LED.changeBlinkPattern(pat='alternate')
		self.IMU.DRV.catchDropMk2()
		self.DRV_LED.changeBlinkPattern(pat='no_alternate')
		self.DRV_GPS_Ser.reconnectSerial()
		self.transmit('Falling!')
		return True

	def purgeByServo(self):
		#パラシュートをパージ
		self.transmit('Purging...by Servo')
		self.DRV_Servo.swing()
		self.DRV_Servo.terminate()

	def Drill(self,gettime):
		self.transmit('Drill Mode Online.')
		self.transmit('Drill Down.')
		self.DRV_Drill.downDrill()
		self.transmit('Spin Start.')
		self.DRV_Drill.spinDrill()
		self.transmit('Drilling Start.')
		self.DRV_Drill.downDrill(10,0)
		time.sleep(gettime)
		self.transmit('Drilling Over.')
		self.DRV_Drill.stopDrill()
		self.transmit('Drill Up.')
		self.DRV_Drill.upDrill(speed=50)
		self.transmit('Drilling Completed.')
		self.DRV_Drill.terminate()

	def face2Goal(self):
		#進行方向を電子コンパスより取得し、ゴールの方向へ旋回
		self.face2(self.DistancetoG['courseS2G'])

	def face2(self,angle):
		#指定の方向へ旋回
		if (angle >= 180) or (angle < -180):
			raise Exception('invalid argument')
		myCourse = angle - self.IMU.getYaw_single()
		if myCourse < -180:
			myCourse += 360
		elif myCourse > 180:
			myCourse -= 360 
		self.transmit('Course:'+str(myCourse))
		if myCourse <= 0 :
			self.transmit('Turn Left.')
			self.DRV_Mortor.turnLeft_St()
			self.IMU.launch6Axis('yaw')
			while True:
				if self.IMU.EA_YAW < myCourse:
					break
		elif myCourse > 0:
			self.transmit('Turn Right')
			self.DRV_Mortor.turnRight_St()
			self.IMU.launch6Axis('yaw')
			while True:
				if self.IMU.EA_YAW < myCourse:
					break

	def updateMoveTime(self,distance):
		#直進する時間を修正する
		return (distance/self.speed)*0.8

	def moveForward(self,movetime,maxdeg=5):
		#直進する、進路がずれた時は旋回して修正する
		#エンコーダーは禁句
		MoveTime = movetime
		self.DRV_GPS_Ser.startMark()
		self.IMU.launch6Axis(mode='pos_yaw')
		moveStart = time.time()
		self.DRV_Mortor.moveForward_St()
		while (time.time() - moveStart) < MoveTime:
			if self.IMU.EA_YAW > maxdeg:
				lossTime = time.time()
				self.DRV_Mortor.turnLeft_St()
				while True:
					if self.IMU.EA_YAW < maxdeg:
						self.DRV_Mortor.moveForward_St()
						MoveTime += time.time() - lossTime
						break
			elif self.IMU.EA_YAW < -(maxdeg):
				lossTime = time.time()
				self.DRV_Mortor.turnLeft_St()
				while True:
					if self.IMU.EA_YAW > -(maxdeg):
						self.DRV_Mortor.moveForward_St()
						MoveTime += time.time() - lossTime
						break
		self.DRV_GPS_Ser.readGPSData(limit=0.5)
		self.DRV_GPS_Ser.stopMark()
		self.DRV_Mortor.stopMotor()
		self.IMU.killThread()
		if  self.IMU.ACC_VX < 1:
			self.escapeFromStack()

	def getRPS(self):
		ang_vec_avr = numpy.rad2deg(self.IMU.ACC_VX/self.diameter)
		ang_vec_diff = self.IMU.YAW_speed*self.roll_coe1
		ang_vec_left = ang_vec_diff + ang_vec_avr
		ang_vec_right = -ang_vec_diff + ang_vec_avr
		self.RPS_L = ang_vec_left/self.roll_coe2
		self.RPS_R = ang_vec_right/self.roll_coe2
		res_dict = {
			'R':self.RPS_R,
			'L':self.RPS_L
		}
		return res_dict

	def fixPWM(self):
		err = self.getRPS()
		eR = err['R'] - self.tagRPS
		eL = err['L'] - self.tagRPS
		PWM_R = self.Kp*(eR+eR*self.IMU.sample_time/self.Ti+eR*self.Td/self.IMU.sample_time)
		PWM_L = self.Kp*(eL+eL*self.IMU.sample_time/self.Ti+eL*self.Td/self.IMU.sample_time)
		self.DRV_Mortor.setSpeed(PWM_R,PWM_L)

	def moveForwardv2(self,movetime,maxdeg=5):
		#直進する、進路がずれた時は旋回して修正する
		#エンコーダーは禁句
		MoveTime = movetime
		self.DRV_GPS_Ser.startMark()
		self.IMU.launch6Axis(mode='pos_yaw')
		moveStart = time.time()
		self.DRV_Mortor.moveForward_St()
		while (time.time() - moveStart) < MoveTime:
			self.fixPWM()
			if self.IMU.EA_YAW > maxdeg:
				lossTime = time.time()
				self.DRV_Mortor.turnLeft_St()
				while True:
					if self.IMU.EA_YAW < maxdeg:
						self.DRV_Mortor.moveForward_St()
						MoveTime += time.time() - lossTime
						break
			elif self.IMU.EA_YAW < -(maxdeg):
				lossTime = time.time()
				self.DRV_Mortor.turnLeft_St()
				while True:
					if self.IMU.EA_YAW > -(maxdeg):
						self.DRV_Mortor.moveForward_St()
						MoveTime += time.time() - lossTime
						break
		self.DRV_GPS_Ser.readGPSData(limit=0.5)
		self.DRV_GPS_Ser.stopMark()
		self.DRV_Mortor.stopMotor()
		self.IMU.killThread()
		if self.IMU.ACC_VX < 1:
			self.escapeFromStack()

	def getLatLng(self,fromLine=False):
		self.LatLngB = self.LatLngN
		if fromLine == True:
			self.LatLngN = self.DRV_GPS_Ser.getPoint(self.DRV_GPS_Ser.markstop)
			self.transmit('got EQA Data from line.')
		else:
			self.LatLngN = self.DRV_GPS_Ser.readGPSData()
			self.transmit('GPS Data Recieved.')
		self.transmit('Latitude:'+str(self.LatLngN['lat'])+',Longitude:'+str(self.LatLngN['lng']))
		return self.LatLngN

	def getLatLngv2(self,use_gps=True):
		#緯度経度を取得する
		if (use_gps == True) or (self.flag_stack == True):
			self.LatLngN = self.DRV_GPS_Ser.readGPSData()
			self.transmit('GPS Data Recieved.')
		else:
			if self.checkLine == True:
				self.LatLngN = self.DRV_GPS_Ser.getPoint(self.DRV_GPS_Ser.markstop)
				self.transmit('got EQA Data from line.')
			else:
				self.LatLngN = self.DRV_GPS_Ser.readGPSData()
				self.transmit('GPS Data Recieved.')
		self.DistancefromB = self.DRV_GPS_Ser.getDistance2(self.LatLngB,self.LatLngN)
		self.DistancetoG = self.DRV_GPS_Ser.getDistance2(self.LatLngN,self.LatLngG)
		if self.flag_stack == True:
			self.DistancefromB['course'] = self.IMU.getYaw_single()
		val = {
			'lat':self.LatLngN['lat'],
			'lng':self.LatLngN['lng'],
			'Distance_to_Goal':self.DistancetoG['distance'],
			'Course_to_Goal':self.DistancetoG['courseS2G'],
			'Distance_from_Previous':self.DistancefromB['distance'],
			'Course_from_Previous':self.DistancefromB['courseS2G']
		}
		self.transmit(str(val))
		return val

	def checkLine(self,limit=3):#いろいろ
		line = self.DRV_GPS_Ser.getLineUTM()
		LatLngB_line = self.DRV_GPS_Ser.getPointfromLine(self.DRV_GPS_Ser.markstart,line)
		LatLngN_line = self.DRV_GPS_Ser.getPointfromLine(self.DRV_GPS_Ser.markstop,line)
		leng = len(self.DRV_GPS_Ser.buffer) - 3
		LatLngB_gps = {
			'lat':self.DRV_GPS_Ser.buffer[0]['lat'],
			'lng':self.DRV_GPS_Ser.buffer[0]['lng']
		}
		LatLngN_gps = {
			'lat':self.DRV_GPS_Ser.buffer[leng]['lat'],
			'lng':self.DRV_GPS_Ser.buffer[leng]['lng']
		}
		course_line = self.DRV_GPS_Ser.getDistance2(LatLngB_line,LatLngN_line)['courseS2G']
		course_gps = self.DRV_GPS_Ser.getDistance2(LatLngB_gps,LatLngN_gps)['courseS2G']
		if abs(course_line - course_gps) > limit:
			return False
		else:
			return True

	def escapeFromStack(self):#スタックから抜け出す
		self.transmit('Stacked!. Escaping...')
		self.getLatLng()
		self.IMU.launch6Axis('yaw')
		self.DRV_Mortor.escapeStack()
		self.IMU.killThread()
		turn_reset = -self.IMU.EA_YAW
		self.DRV_Mortor.moveForward(2)
		self.IMU.launch6Axis('yaw')
		if turn_reset < 0:
			self.DRV_Mortor.turnLeft_St()
			while True:
				if self.IMU.EA_YAW*self.TURN_Coefficient < turn_reset:
					self.DRV_Mortor.stopMotor()
					self.transmit('Turned(deg):'+str(self.IMU.EA_YAW))
					break
		else:
			self.DRV_Mortor.turnRight_St()
			while True:
				if self.IMU.EA_YAW*self.TURN_Coefficient > turn_reset:
					self.DRV_Mortor.stopMotor()
					self.transmit('Turned(deg):'+str(self.IMU.EA_YAW))
					break
		self.IMU.killThread()
		self.flag_stack = True

	def getDistance(self):
		self.DistancefromB = self.DRV_GPS_Ser.getDistance2(self.LatLngB,self.LatLngN)
		self.DistancetoG = self.DRV_GPS_Ser.getDistance2(self.LatLngN,self.LatLngG)
		return [self.DistancefromB,self.DistancetoG]

	def getDistancev2(self,use_gps=False):
		if (use_gps == True) or (self.flag_stack == True):
			self.getLatLng(False)
		else:
			self.getLatLng(self.checkLine())
		self.DistancefromB = self.DRV_GPS_Ser.getDistance2(self.LatLngB,self.LatLngN)
		self.DistancetoG = self.DRV_GPS_Ser.getDistance2(self.LatLngN,self.LatLngG)
		if self.flag_stack == True:
			self.DistancefromB['course'] = self.IMU.getYaw_single()
		return [self.DistancefromB,self.DistancetoG]

	def detectTargetfromPicture(self):#写真を撮って目標を識別する
		while True:
			self.transmit('Taking a Picture...')
			self.DRV_Camera.getImage()
			self.transmit('Detecting...')
			var = self.DRV_Camera.detectTarget()
			if var == False:
				self.getLatLngv2()
				self.face2Goal()
				continue
			else:
				self.transmit('Target Acquired.')
				if self.DRV_Camera.judge():
					self.transmit('Target is Front.')
					self.DRV_Mortor.moveForward(1)
					self.DistancetoG = self.DRV_GPS_Ser.readGPSData()
					break
				else:
					if var < 512:
						self.transmit('Target is Left.')
						self.DRV_Mortor.turnLeft(0.2)
						continue
					else:
						self.transmit('Target is Right.')
						self.DRV_Mortor.turnRight(0.2)
						continue
 
	def calibrateAG(self,num=1000):
		self.IMU.DRV.offsetAccelX = 0
		self.IMU.DRV.offsetAccelY = 0
		self.IMU.DRV.offsetAccelZ = 0
		self.IMU.DRV.offsetGyroX = 0
		self.IMU.DRV.offsetGyroY = 0
		self.IMU.DRV.offsetGyroZ = 0
		acc = [0,0,0]
		gyr = [0,0,0]
		for i in range(num):
			vala = self.IMU.DRV.readAccel()
			valg = self.IMU.DRV.readGyro()
			acc[0] += vala[0]
			acc[1] += vala[1]
			acc[2] += (vala[2] + 1)
			gyr[0] += valg[0]
			gyr[1] += valg[1]
			gyr[2] += valg[2]
		acc[0] /= num
		acc[1] /= num
		acc[2] /= num
		gyr[0] /= num
		gyr[1] /= num
		gyr[2] /= num
		with open('./config.json','r') as file:
			conf = json.load(file)
		conf['config']['control']['imu']['calibration']['accel']['x'] = acc[0]
		conf['config']['control']['imu']['calibration']['accel']['y'] = acc[1]
		conf['config']['control']['imu']['calibration']['accel']['z'] = acc[2]
		conf['config']['control']['imu']['calibration']['gyro']['x'] = gyr[0]
		conf['config']['control']['imu']['calibration']['gyro']['y'] = gyr[1]
		conf['config']['control']['imu']['calibration']['gyro']['z'] = gyr[2]
		with open('./config.json','w') as file:
			json.dump(conf,file,indent=4)

	def calibrateM(self,num=1000,path='MagData.csv',init=True):
		self.DRV_Mortor.turnRight_St()
		self.IMU.DRV.MagOffsetX = 0
		self.IMU.DRV.MagOffsetY = 0
		self.IMU.DRV.MagOffsetZ = 0
		if init == True:
			if os.path.isfile(path):
				os.remove(path)
			else:
				self.transmit("the data file does not exist. making a new file..."+str(path))
			
		with open(path,'a') as file:
			writer = csv.writer(file)
			for i in range(num):
				val = self.IMU.DRV.readMag()
				writer.writerow([val[0],val[1],val[2]])
		data = numpy.loadtxt("MagData.csv", delimiter=',')
		x = data[:,[0]]
		y = data[:,[1]]
		z = data[:,[2]] 
		n = len(x)    
		x2 = numpy.power(x,2)
		y2 = numpy.power(y,2)
		z2 = numpy.power(z,2)
		xy = x*y
		xz = x*z
		yz = y*z
		E = -x*(x2+y2+z2)
		F = -y*(x2+y2+z2)
		G = -z*(x2+y2+z2)
		H =   -(x2+y2+z2)
		x = numpy.sum(x)
		y = numpy.sum(y)
		z = numpy.sum(z)
		x2 = numpy.sum(x2)
		y2 = numpy.sum(y2)
		z2 = numpy.sum(z2)
		xy = numpy.sum(xy)
		xz = numpy.sum(xz)
		yz = numpy.sum(yz)    
		E = numpy.sum(E)
		F = numpy.sum(F)
		G = numpy.sum(G)
		H = numpy.sum(H)
		K = numpy.array([  
						[x2,xy,xz,x],
						[xy,y2,yz,y],
						[xz,yz,z2,z],
						[x,y,z,n]
						])
		L = numpy.array([E,F,G,H])
		P = numpy.dot(numpy.linalg.inv(K),L)
		A = P[0]
		B = P[1]
		C = P[2]
		D = P[3]
		x0 = (-1/2)* A
		y0 = (-1/2)* B
		z0 = (-1/2)* C
		r  = pow(pow(x0,2)+pow(y0,2)+pow(z0,2)-D,1/2)
		with open('./config.json','r') as file:
			conf = json.load(file)
			conf['config']['control']['imu']['calibration']['magnet']['x'] = x0
			conf['config']['control']['imu']['calibration']['magnet']['y'] = y0
			conf['config']['control']['imu']['calibration']['magnet']['z'] = z0
			conf['config']['control']['imu']['calibration']['magnet']['r'] = r
		with open('./config.json','w') as file:
			json.dump(conf,file,indent=4)

	def loadCalibration(self,accel=True,gyro=True,magnet=True):
		with open('./config.json','r') as file:
			conf = json.load(file)
			if accel == True:
				self.IMU.DRV.offsetAccelX = conf['config']['control']['imu']['calibration']['accel']['x']
				self.IMU.DRV.offsetAccelY = conf['config']['control']['imu']['calibration']['accel']['y']
				self.IMU.DRV.offsetAccelZ = conf['config']['control']['imu']['calibration']['accel']['x']
			if gyro == True:
				self.IMU.DRV.offsetGyroX = conf['config']['control']['imu']['calibration']['gyro']['x']
				self.IMU.DRV.offsetGyroY = conf['config']['control']['imu']['calibration']['gyro']['y']
				self.IMU.DRV.offsetGyroZ = conf['config']['control']['imu']['calibration']['gyro']['z']
			if magnet == True:
				self.IMU.DRV.MagOffsetX = conf['config']['control']['imu']['calibration']['magnet']['x']
				self.IMU.DRV.MagOffsetY = conf['config']['control']['imu']['calibration']['magnet']['y']
				self.IMU.DRV.MagOffsetZ = conf['config']['control']['imu']['calibration']['magnet']['z']

	def terminate(self):
		self.DRV_LED.terminate()
		self.DRV_Mortor.terminate()
		self.DRV_Servo.terminate()
		self.DRV_GPS_Ser.stopGps()
		self.DRV_GPS_Ser.stopSerial()

	

