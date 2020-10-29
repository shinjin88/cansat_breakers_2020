import serial
import micropyGPS
import numpy
import pyproj
import time
import threading
import cython


class Gps_Serial:

    threadflag = False
    logflag = False
    buff = {}
    lognum = 0
    timestamp = 0
    markstart = 0
    markstop = 0

    def __init__(self,str ser='/dev/serial0',int baudrate=9600,int timeout=10,int timezone=9,str fmt='dd',str log='./log/log.txt'):
        self.s = serial.Serial(ser,baudrate=baudrate,timeout=timeout)
        self.gps = micropyGPS.MicropyGPS(timezone,fmt)
        self.gps.start_logging(log)

    def transmitData(self,str var='can you hear me?'):
        cdef str payload = (str(var)+('\n')).encode('utf-8')
        self.s.write(payload)

    def stopSerial(self):
        self.s.close()

    def reconnectSerial(self):
        self.s.open()
                
    def getGpsData(self):
        self.s.readline()#1回目は読み捨て
        cdef str sentence
        while self.threadflag:
            sentence = self.s.readline().decode('utf-8')       
            #if(sentence[0] != '$'):#←これいる？
            #    continue
            for x in sentence:
                self.gps.update(x)
                self.timestamp = time.time()
                if self.logflag == True:
                    self.buff[self.lognum] = {
                        'timestamp':self.timestamp,
                        'lat':self.gps.latitude,
                        'lng':self.gps.longitude
                    }
                    self.lognum += 1

    def startMark(self):
        self.markstart = time.time()
        self.logflag = True
        self.lognum = 0
        self.buff = {}

    def stopMark(self):
        self.markstop = time.time()
        self.logflag = False
        self.buff['markstart'] = self.markstart
        self.buff['markstop'] = self.markstop
        return self.buff

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
        self.gps.stop_logging()

    @classmethod
    def getDistance(cls,double lat0=0,double lng0=0,double lat1=0,double lng1=0):#Vincentry法
        #差異が無ければ0
        if numpy.isclose(lat0,lat1) and numpy.isclose(lng0,lng1):
            return {
                'distance':0,
                'courseS2G':0,
                'courseG2S':0
            }

        #WGS84
        cdef double a = 6378137.0#赤道半径
        cdef double f = 1/298.257223563#扁平率
        #短軸半径
        cdef double b = (1-f)*a
        #ラジアンへ変換
        cdef double rlat0 = numpy.deg2rad(lat0)
        cdef double rlng0 = numpy.deg2rad(lng0)
        cdef double rlat1 = numpy.deg2rad(lat1)
        cdef double rlng1 = numpy.deg2rad(lng1)
        #更成緯度
        cdef double U0 = numpy.arctan((1-f)*numpy.tan(rlat0))
        cdef double U1 = numpy.arctan((1-f)*numpy.tan(rlat1))
        #2点間の経度差  
        cdef double L = rlng0-rlng1
        #補助球上の経度(初期化)
        cdef double Lambda = L

        cdef double sinU0 = numpy.sin(U0)
        cdef double sinU1 = numpy.sin(U1)
        cdef double cosU0 = numpy.cos(U0)
        cdef double cosU1 = numpy.cos(U1)

        cdef double sinLmd
        cdef double cosLmd
        cdef double sins
        cdef double coss
        cdef double sigma
        cdef double sina
        cdef double cos2a
        cdef double cos2dm
        cdef double C
        cdef double Lambda_prev
        for _ in range(1000):     
            sinLmd = numpy.sin(Lambda)
            cosLmd = numpy.cos(Lambda)
        
            sins = ((cosU1*sinLmd)**2+(cosU0*sinU1-sinU0*cosU1*cosLmd)**2)**(1/2)
            coss = sinU0*sinU1+cosU0*cosU1*cosLmd

            sigma = numpy.arctan2(sins,coss)

            sina = cosU0*cosU1*sinLmd/sins
            cos2a = 1-sina**2

            cos2dm = coss-(2*sinU0*sinU1/cos2a)

            C = f*(cos2a)*(4+f*(4-3*cos2a))/16
            Lambda_prev = Lambda
            Lambda = L+(1-C)*f*sina*(sigma+C*sins*(cos2dm+C*coss*(-1+2*coss**2)))
            #精度の指定
            if abs(Lambda-Lambda_prev)<=1e-12:
                break
            
    
        cdef double u2=cos2a*(a**2-b**2)/b**2
        cdef double A=1+u2*(4096+u2*(-768+u2*(320-175*u2)))/16384
        cdef double B=u2*(256+u2*(-128+u2*(74-47*u2)))/1024
    
        cdef double deltas=B*sins*(cos2dm+B*(coss*(-1+2*cos2dm**2)-B*cos2dm*(-3+4*sins**2)*(-3+4*cos2dm**2)/6)/4)

        cdef double distance=b*A*(sigma-deltas)
        cdef double course0to1=numpy.rad2deg(numpy.arctan2(cosU1*sinLmd,cosU0*sinU1-sinU0*cosU1*cosLmd))
        cdef double course1to0=numpy.rad2deg(numpy.arctan2(cosU0*sinLmd,-sinU0*cosU1+cosU0*sinU1*cosLmd))

        return {
            'distance':distance,
            'courseS2G':course0to1,
            'courseG2S':course1to0
        }

    @classmethod
    def getDistance2(cls,dict latlng0,dict latlng1):
        return cls.getDistance(latlng0['lat'],latlng0['lng'],latlng1['lat'],latlng1['lng'])

    @classmethod
    def getUTMZone(cls,double lng):
        return (int(lng//6) + 31)

    @classmethod
    def getUTM(cls,double lat,double lng):
        cdef int myzone = cls.getUTMZone(lng)
        cdef object converter = pyproj.Proj(proj='utm',zone=myzone,elps='WGS84')
        utmx,utmy = converter(lng,lat)
        if lat < 0:
            utmy += 10000000
        return {
            'zone':myzone,
            'utmx':utmx,
            'utmy':utmy
        }

    @classmethod
    def UTM2EQA(cls,double utmx,double utmy,int zone,str hemisphere='N'):
        cdef double x = utmx
        cdef double y = utmy
        cdef int z = zone
        if hemisphere == 'S':
            y -= 10000000
        cdef object converter = pyproj.Proj(proj='utm',zone=z,elps='WGS84')
        lng,lat = converter(x,y,inverse=True)
        return {
            'lat':lat,
            'lng':lng
        }

    def getLineUTM(self):
        cdef dict buff = self.buff
        cdef int length = len(buff) - 2 
        cdef double sum_t = 0
        cdef double sum_t2 = 0
        cdef double sum_tx = 0
        cdef double sum_ty = 0
        cdef double sum_x = 0
        cdef double sum_y = 0
        cdef int zone = 0
        cdef dict utm
        cdef object converter
        try:     
            for i in range(length):
                utm = self.getUTM(buff[i]['lat'],buff[i]['lng'])
                if zone != utm['zone']:
                    converter = pyproj.Proj(proj='utm',zone=zone,elps='WGS84')
                    utmx,utmy = converter(buff[i]['lng'],buff[i]['lat'])
                    if buff[i]['lat'] < 0:
                        utmy += 10000000
                    utm =  {
                        'zone':zone,
                        'utmx':utmx,
                        'utmy':utmy
                    }
                #zone = utm['zone']
                t = (buff[i]['timestamp'] - buff['markstart'])
                sum_t += t       
                sum_t2 += t**2
                sum_tx += t*utm['utmx']
                sum_ty += t*utm['utmy']
                sum_x += utm['utmx']
                sum_y += utm['utmy']
        except KeyError:
            pass
        cdef double dt = (length*sum_t2 - sum_t**2)
        cdef double ax = (length*sum_tx - sum_t*sum_x)/dt
        cdef double ay = (length*sum_ty - sum_t*sum_y)/dt
        cdef double bx = (sum_t2*sum_x - sum_tx*sum_t)/dt
        cdef double by = (sum_t2*sum_y - sum_ty*sum_t)/dt
        #X = ax*t + bx
        #Y = ay*t + by
        cdef str hemisphere
        if buff[0]['lat'] < 0:
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
            'start':buff['markstart'],
            'stop':buff['markstop']
        }

    def getPoint(self,double timestamp):
        cdef dict val = self.getLineUTM()
        cdef double t = timestamp - val['start']
        cdef double y = val['ay']*t + val['by']
        cdef double x = val['ax']*t + val['bx']
        return self.UTM2EQA(x,y,val['zone'],val['hemisphere'])

    def getPointfromLine(self,double timestamp,dict line):
        cdef double t = timestamp - line['start']
        cdef double y = line['ay']*t + line['by']
        cdef double x = line['ax']*t + line['bx']
        return self.UTM2EQA(x,y,line['zone'],line['hemisphere'])


    
