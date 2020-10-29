import pigpio
import time
import threading

class Gpio:

	def __init__(self,instance=None):
		if instance == None:
			self.gpio = pigpio.pi()
		else:
			self.gpio = instance

	def setPinOut(self,gpionum):
		self.gpio.set_mode(gpionum,pigpio.OUTPUT)

	def setPinIn(self,gpionum):
		self.gpio.set_mode(gpionum,pigpio.INPUT)

	def setServo(self,gpionum,pulse_width=500):#500~2500
		self.gpio.set_servo_pulsewidth(gpionum,pulse_width)

	def setPWM(self,gpionum,frequency=1000,range_=255,dutycycle=128):
		self.gpio.set_PWM_frequency(gpionum,frequency)
		self.gpio.set_PWM_range(gpionum,range_)
		self.gpio.set_PWM_dutycycle(gpionum,dutycycle)

	def changeDutycycle(self,gpionum,dutycycle):
		self.gpio.set_PWM_dutycycle(gpionum,dutycycle)

	def setInterrupt(self,gpionum,target,edge='FALLING'):
		self.setPinIn(gpionum)
		self.gpio.set_pull_up_down(gpionum,pigpio.PUD_UP)
		if edge == 'FALLING':
			var = pigpio.FALLING_EDGE
		elif edge == 'RISING':
			var = pigpio.RISING_EDGE
		elif edge == 'EITHER':
			var = pigpio.EITHER_EDGE
		else:
			raise Exception('Invalid EDGE Value')
		self.cb = self.gpio.callback(gpionum,var,target)
		return self.cb

	def toggleOn(self,gpionum):
		self.gpio.write(gpionum,1)

	def toggleOff(self,gpionum):
		self.gpio.write(gpionum,0)

	def terminate(self):
		self.gpio.stop()

class MotorControl:

	def __init__(self,PinR1,PinR2,pinR_VREF,PinL1,PinL2,pinL_VREF):
		self.gpio = Gpio()
		self.R1 = PinR1
		self.R2 = PinR2
		self.R_VREF = pinR_VREF
		self.L1 = PinL1
		self.L2 = PinL2
		self.L_VREF = pinL_VREF
		self.gpio.setPinOut(PinR1)
		self.gpio.setPinOut(PinR2)
		self.gpio.setPWM(pinR_VREF)
		self.gpio.setPinOut(PinL1)
		self.gpio.setPinOut(PinL2)
		self.gpio.setPWM(pinL_VREF)
		
	def spinMotor(self,mot):
		if(mot[0]=='1'):
			self.gpio.toggleOn(self.R1)
			self.gpio.toggleOff(self.R2)
		elif(mot[0]=='2'):
			self.gpio.toggleOff(self.R1)
			self.gpio.toggleOn(self.R2)
		elif(mot[0]=='0'):
			self.gpio.toggleOff(self.R1)
			self.gpio.toggleOff(self.R2)
		
		if(mot[1]=='1'):
			self.gpio.toggleOn(self.L1)
			self.gpio.toggleOff(self.L2)
		elif(mot[1]=='2'):
			self.gpio.toggleOff(self.L1)
			self.gpio.toggleOn(self.L2)
		elif(mot[1]=='0'):
			self.gpio.toggleOff(self.L1)
			self.gpio.toggleOff(self.L2)
			
	def stopMotor(self):
				#self.gpio.write(self.R1,0)
				#self.gpio.write(self.R2,0)
				#self.gpio.write(self.L1,0)
				#self.gpio.write(self.L2,0)
		self.spinMotor('00')

	def setSpeed(self,vref_R=80,vref_L=80):
		vrefR = 2.55*vref_R
		vrefL = 2.55*vref_L
		if ((vrefL > 0 or (vrefL < 100)) or ((vrefR > 0) or (vrefR < 100)):
			raise Exception('invalid vref setting value.')
		self.gpio.setPWM(self.R_VREF,vrefR)
		self.gpio.setPWM(self.L_VREF,vrefL)

	def turnLeft_St(self):
		print('turn left')
		self.spinMotor('10')

	def turnRight_St(self):
		print('turn right')
		self.spinMotor('01')
	
	def turnLeft(self,Time):
		print('turn left')
		self.spinMotor('10')
		time.sleep(Time)
		self.stopMotor()
		return True

	def turnRight(self,Time):
		print('turn right')
		self.spinMotor('01')
		time.sleep(Time)
		self.stopMotor()
		return True

	def moveForward(self,Time):
		self.spinMotor('11')
		time.sleep(Time)
		self.stopMotor()
		return True

	def moveForward_St(self):
		self.spinMotor('11')

	def escapeStack(self):
		self.spinMotor('02')
		time.sleep(2)
		self.stopMotor()
		self.spinMotor('10')
		time.sleep(2)
		self.stopMotor()
		return True
	
	def testMotor(self):
		self.spinMotor('12')
		time.sleep(2)
		self.stopMotor()
		self.spinMotor('21')
		time.sleep(2)
		self.stopMotor()

	def terminate(self):
		self.gpio.terminate()

class Servo:
	def __init__(self,gpionum,waittime=0.5):
		self.gpio = Gpio()
		self.pin = gpionum
		self.waittime = 0.5

	def roll(self,pwidth=500):
		self.gpio.gpio.set_servo_pulsewidth(self.pin,pwidth)
		time.sleep(self.waittime)

	def swing(self):
		for i in range(3):
			self.roll(500)
			self.roll(2500)

	def terminate(self):
		self.gpio.terminate()


class Drill:

	table_dict = {                          #データ変換テーブル
			'00':{'00':0,'01':1,'10':-1,'11':-2},#11はエラー
			'01':{'00':-1,'01':0,'10':-2,'11':1},#10はエラー
			'10':{'00':1,'01':-2,'10':0,'11':-1},#01はエラー
			'11':{'00':-2,'01':-1,'10':1,'11':0} #00はエラー
		}

	previos_data = '00'#前回の値
	value = 0

	def __init__(self,pinElvA,pinElvB,pinElv_VREF,pinDA,pinDB,pinEA,pinEB,rad=10,maxleng=100):
		self.gpio = Gpio()
		self.PELA = pinElvA
		self.PELB = pinElvB
		self.PEL_VREF = pinElv_VREF
		self.DA = pinDA
		self.DB = pinDB
		self.PEA = pinEA
		self.PEB = pinEB
		self.radius = rad
		self.length = maxleng
		self.gpio.setPinOut(pinElvA)
		self.gpio.setPinOut(pinElvB)
		self.gpio.setPinOut(pinDA)
		self.gpio.setPinOut(pinDB)
		self.cb1 = self.gpio.setInterrupt(pinEA,self.recData,'RISING')
		self.cb2 = self.gpio.setInterrupt(pinEB,self.recData,'RISING')

	def upDrill(self,leng=None,speed=100):
		spd = speed/100
		val = 2000*spd + 500
		if leng == None:
			leng = self.length-20
		elif leng > self.length:
			raise Exception('Invalid Argument')
		elif leng < 0:
			raise Exception('Invalid Argument')
		self.gpio.setPWM(self.PEL_VREF,val)
		self.gpio.toggleOn(self.PELA)
		self.gpio.toggleOff(self.PELB)
		while True:
			val = (self.deg/360)*3.14*self.radius
			if (val >= leng):
				self.gpio.toggleOff(self.PELA)
				self.gpio.toggleOff(self.PELB)
				break 

	def downDrill(self,leng=None,speed=100):
		spd = speed/100
		val = 2000*spd + 500
		if leng == None:
			leng = 0
		elif leng > self.length:
			raise Exception('Invalid Argument')
		elif leng < 0:
			raise Exception('Invalid Argument')
		self.gpio.setPWM(self.PEL_VREF,val)
		self.gpio.toggleOff(self.PELA)
		self.gpio.toggleOn(self.PELB)
		while True:
			val = (self.deg/360)*3.14*self.radius
			if (val <= leng):
				self.gpio.toggleOff(self.PELA)
				self.gpio.toggleOff(self.PELB)
				break 

	def spinDrill(self,invert=False):
		if invert == True:
			self.gpio.toggleOff(self.DA)
			self.gpio.toggleOn(self.DB)
		else:
			self.gpio.toggleOn(self.DA)
			self.gpio.toggleOff(self.DB)

	def stopDrill(self):
		self.gpio.toggleOff(self.DA)
		self.gpio.toggleOff(self.DB)

	def terminate(self):
		self.gpio.terminate()

	def recData(self,gpio,level,tick):#本当はもっと軽い処理にしたいのでこの辺りは改良の余地ありか？
		follow_data = '00'#初期化
		if gpio == self.PEA:#どちらのピンに割り込みが入ったかで分岐
			follow_data = '{0}{1}'.format('1',self.previos_data[1])
		elif gpio == self.PEB:
			follow_data = '{0}{1}'.format(self.previos_data[0],'1')
		self.value += self.table_dict[self.previos_data][follow_data]#取得値を変換して足す
		if self.value == -2:
			self.value = 0
		self.previos_data = follow_data#値を保存
		self.deg = 12*self.value#度数法に変換
		return self.deg

class LED:
	flag = False
	alt = True
	led1_status = True
	led2_status = False
	interval = 0.5

	def __init__(self,LED1,LED2):
		self.gpio = Gpio()
		self.led1 = LED1
		self.led2 = LED2

	def toggleOn(self,LED1=True,LED2=True):
		if LED1 == True:
			self.gpio.toggleOn(self.led1)
		if LED2 == True:
			self.gpio.toggleOn(self.led2)

	def toggleOff(self,LED1=True,LED2=True):
		if LED1 == True:
			self.gpio.toggleOff(self.led1)
		if LED2 == True:
			self.gpio.toggleOff(self.led2)

	def blink(self,LED1=True,LED2=True,alt=True,interval=0.5):
		if ((LED1 == False) and (alt == True)) or ((LED2 == False) and (alt == True)):
			raise Exception('Invalid Argument')
		self.flag = True
		self.alt = alt
		self.interval = interval
		if self.alt == True:
			self.led1_status = True
			self.led2_status = False
		else:
			if LED1 == False:
				self.led1_status = False
			if LED2 == False:
				self.led2_status = False 
		while self.flag == True:    
			self.toggleOn(self.led1_status,self.led2_status)
			time.sleep(interval)
			if self.alt == True:
				if self.led1_status == True:
					self.led1_status = False
				else:
					self.led1_status = True
				if self.led2_status == True:    
					self.led2_status = False
				else:
					self.led2_status = True
				continue 
			self.toggleOff()
			time.sleep(interval)

	def startBlink(self,LED1=True,LED2=True,alt=True,interval=0.5):
		self.thread = threading.Thread(target=self.blink,args=(LED1,LED2,alt,interval))
		self.thread.daemon = True
		self.thread.start()

	def stopBlink(self):
		self.flag = False

	def changeBlinkPattern(self,pat='alternate'):
		if pat == 'alternate':
			self.led1_status = True
			self.led2_status = False
			self.alt = True
		elif pat == 'no_alternate':
			self.led1_status = True
			self.led2_status = True
			self.alt = False
		elif pat == 'led1_only':
			self.alt = False
			self.led1_status = True
			self.led2_status = False
		elif pat == 'led2_only':
			self.alt == False
			self.led1_status = False
			self.led2_status = True

	def terminate(self):
		self.gpio.terminate()

		
		





