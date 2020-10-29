import serial
import micropyGPS
import numpy
import pyproj
import time
import threading


class Gps_Serial:

    threadflag = False
    logflag = False
    buffer = {}
    lognum = 0
    timestamp = 0

    def __init__(self,ser='/dev/serial0',baudrate=9600,timeout=10,timezone=9,fmt='dd',log='./log/log.txt'):
        self.s = serial.Serial(ser,baudrate=baudrate,timeout=timeout)
        self.gps = micropyGPS.MicropyGPS(timezone,fmt)
        self.gps.start_logging(log)

    def transmitData(self,var='can you hear me?'):
        payload = (str(var)+('\n')).encode('utf-8')
        self.s.write(payload)

    def stopSerial(self):
        self.s.close()

    def reconnectSerial(self):
        self.s.open()
                
    def getGpsData(self):
        self.s.readline()#1回目は読み捨て
        while self.threadflag:
            sentence = self.s.readline().decode('utf-8')       
            #if(sentence[0] != '$'):#←これいる？
            #    continue
            for x in sentence:
                self.gps.update(x)
                self.timestamp = time.time()
                if self.logflag == True:
                	self.buffer[self.lognum] = {
                		'timestamp':self.timestamp,
                		'lat':self.gps.latitude,
                		'lng':self.gps.longitude
                	}
                	self.lognum += 1

    def startMark(self):
        self.markstart = time.time()
        self.logflag = True
        self.lognum = 0
        self.buffer = {}

    def stopMark(self):
        self.markstop = time.time()
        self.logflag = False
        self.buffer['markstart'] = self.markstart
        self.buffer['markstop'] = self.markstop
        return self.buffer

    def readGPSData(self,limit=0.2):
        while True:
            now = time.time()
            if abs(now-self.timestamp) < limit:
                return {
                    'lat':self.gps.latitude,
                    'lng':self.gps.longitude
                } 
              
    def launchGps(self):
        self.threadflag = True
        self.gpsthread = threading.Thread(target=self.getGpsData,args = ())
        self.gpsthread.daemon = True#デーモン化　
        self.gpsthread.start()

    def stopGps(self):
        self.threadflag = False

    @classmethod
    def getDistance(cls,lat0=0,lng0=0,lat1=0,lng1=0):#Vincentry法
        #差異が無ければ0
        if numpy.isclose(lat0,lat1) and numpy.isclose(lng0,lng1):
            return {
                'distance':0,
                'courseS2G':0,
                'courseG2S':0
            }
    
        #WGS84
        a = 6378137.0#赤道半径
        f = 1/298.257223563#扁平率
        #短軸半径
        b = (1-f)*a
        #ラジアンへ変換
        rlat0 = numpy.deg2rad(lat0)
        rlng0 = numpy.deg2rad(lng0)
        rlat1 = numpy.deg2rad(lat1)
        rlng1 = numpy.deg2rad(lng1)
        #更成緯度
        U0 = numpy.arctan((1-f)*numpy.tan(rlat0))
        U1 = numpy.arctan((1-f)*numpy.tan(rlat1))
        #2点間の経度差  
        L = rlng0-rlng1
        #補助球上の経度(初期化)
        Lambda=L

        sinU0 = numpy.sin(U0)
        sinU1 = numpy.sin(U1)
        cosU0 = numpy.cos(U0)
        cosU1 = numpy.cos(U1)
    
        for i in range(1000):
            sinLmd=numpy.sin(Lambda)
            cosLmd=numpy.cos(Lambda)
        
            sins=((cosU1*sinLmd)**2+(cosU0*sinU1-sinU0*cosU1*cosLmd)**2)**(1/2)
            coss=sinU0*sinU1+cosU0*cosU1*cosLmd

            sigma=numpy.arctan2(sins,coss)

            sina=cosU0*cosU1*sinLmd/sins
            cos2a=1-sina**2

            cos2dm=coss-(2*sinU0*sinU1/cos2a)

            C=f*(cos2a)*(4+f*(4-3*cos2a))/16
            Lambda_prev=Lambda
            Lambda=L+(1-C)*f*sina*(sigma+C*sins*(cos2dm+C*coss*(-1+2*coss**2)))
            #精度の指定
            if abs(Lambda-Lambda_prev)<=1e-12:
                break
    
        u2=cos2a*(a**2-b**2)/b**2
        A=1+u2*(4096+u2*(-768+u2*(320-175*u2)))/16384
        B=u2*(256+u2*(-128+u2*(74-47*u2)))/1024
    
        deltas=B*sins*(cos2dm+B*(coss*(-1+2*cos2dm**2)-B*cos2dm*(-3+4*sins**2)*(-3+4*cos2dm**2)/6)/4)

        distance=b*A*(sigma-deltas)
        course0to1=numpy.rad2deg(numpy.arctan2(cosU1*sinLmd,cosU0*sinU1-sinU0*cosU1*cosLmd))
        course1to0=numpy.rad2deg(numpy.arctan2(cosU0*sinLmd,-sinU0*cosU1+cosU0*sinU1*cosLmd))

        return {
            'distance':distance,
            'courseS2G':course0to1,
            'courseG2S':course1to0
        }

    @classmethod
    def getDistance2(cls,latlng0,latlng1):
        return cls.getDistance(latlng0['lat'],latlng0['lng'],latlng1['lat'],latlng1['lng'])

    @classmethod
    def getUTMZone(cls,lng):
        return (int(lng//6) + 31)

    @classmethod
    def getUTM(cls,lat,lng):
        myzone = cls.getUTMZone(lng)
        converter = pyproj.Proj(proj='utm',zone=myzone,elps='WGS84')
        utmx,utmy = converter(lng,lat)
        if lat < 0:
            utmy += 10000000
        return {
            'zone':myzone,
            'utmx':utmx,
            'utmy':utmy
        }

    @classmethod
    def UTM2EQA(cls,utmx,utmy,zone,hemisphere='N'):
        x = utmx
        y = utmy
        z = zone
        if hemisphere == 'S':
            y -= 10000000
        converter = pyproj.Proj(proj='utm',zone=z,elps='WGS84')
        lng,lat = converter(x,y,inverse=True)
        return {
            'lat':lat,
            'lng':lng
        }

    def getLineUTM(self,buffer=None):
        if buffer == None:
            buffer = self.buffer
        length = len(buffer) - 2 
        sum_t = 0
        sum_t2 = 0
        sum_tx = 0
        sum_ty = 0
        sum_x = 0
        sum_y = 0
        zone = None
        try:     
            for i in range(length):
                utm = self.getUTM(buffer[i]['lat'],buffer[i]['lng'])
                if zone != utm['zone']:
                    if zone == None:
                        pass
                    else:
                        converter = pyproj.Proj(proj='utm',zone=zone,elps='WGS84')
                        utmx,utmy = converter(buffer[i]['lng'],buffer[i]['lat'])
                        if buffer[i]['lat'] < 0:
                            utmy += 10000000
                        utm =  {
                            'zone':zone,
                            'utmx':utmx,
                            'utmy':utmy
                        }
                #zone = utm['zone']
                t = (buffer[i]['timestamp'] - buffer['markstart'])
                sum_t += t       
                sum_t2 += t**2
                sum_tx += t*utm['utmx']
                sum_ty += t*utm['utmy']
                sum_x += utm['utmx']
                sum_y += utm['utmy']
        except KeyError:
            pass
        dt = (length*sum_t2 - sum_t**2)
        ax = (length*sum_tx - sum_t*sum_x)/dt
        ay = (length*sum_ty - sum_t*sum_y)/dt
        bx = (sum_t2*sum_x - sum_tx*sum_t)/dt
        by = (sum_t2*sum_y - sum_ty*sum_t)/dt
        #X = ax*t + bx
        #Y = ay*t + by
        if buffer[0]['lat'] < 0:
            hemisphere ='S'
        else:
            hemisphere = 'N'

        return {
            'ax':ax,
            'bx':bx,
            'ay':ay,
            'by':by,
            'zone':utm['zone'],
            'hemisphere':hemisphere,
            'start':buffer['markstart'],
            'stop':buffer['markstop']
        }

    """def getLineUTM(self):
        length = len(self.buffer)
        sum_t = 0
        sum_t2 = 0
        sum_tx = 0
        sum_ty = 0
        sum_x = 0
        sum_y = 0
        zone = None     
        for i in range(length):
            utm = self.getUTM(self.buffer[i]['lat'],self.buffer[i]['lng'])
            if zone != utm['zone']:
                if zone == None:
                    pass
                else:
                    converter = pyproj.Proj(proj='utm',zone=zone,elps='WGS84')
                    utmx,utmy = converter(self.buffer[i]['lng'],self.buffer[i]['lat'])
                    if self.buffer[i]['lat'] < 0:
                        utmy += 10000000
                    utm =  {
                        'zone':zone,
                        'utmx':utmx,
                        'utmy':utmy
                    }
            #zone = utm['zone']
            t = (self.buffer[i]['timestamp'] - self.markstart)
            sum_t += t       
            sum_t2 += t**2
            sum_tx += t*utm['utmx']
            sum_ty += t*utm['utmy']
            sum_x += utm['utmx']
            sum_y += utm['utmy']
        dt = (length*sum_t2 - sum_t**2)
        ax = (length*sum_tx - sum_t*sum_x)/dt
        ay = (length*sum_ty - sum_t*sum_y)/dt
        bx = (sum_t2*sum_x - sum_tx*sum_t)/dt
        by = (sum_t2*sum_y - sum_ty*sum_t)/dt
        #X = ax*t + bx
        #Y = ay*t + by
        return {
            'ax':ax,
            'bx':bx,
            'ay':ay,
            'by':by,
            'start':self.markstart
        }"""

    def getPoint(self,timestamp,buffer=None):
        val = self.getLineUTM(buffer)
        t = timestamp - val['start']
        y = val['ay']*t + val['by']
        x = val['ax']*t + val['bx']
        return self.UTM2EQA(x,y,val['zone'],val['hemisphere'])

    def getPointfromLine(self,timestamp,line):
        t = timestamp - line['start']
        y = line['ay']*t + line['by']
        x = line['ax']*t + line['bx']
        return self.UTM2EQA(x,y,line['zone'],line['hemisphere'])


    
