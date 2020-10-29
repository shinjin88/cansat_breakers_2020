import picamera
import cv2
import time
#from DRV_GPS_Serial import Gps_Serial

class Camera:
    serFlag = False
    def __init__(self,Mx=1024,My=768,roll=90,path='./img/image_',opath='./img/image~',format_='.jpg',cascadepath='cascade_traffic_cone.xml',target_height=0.8):
        self.Mx = Mx
        self.My = My
        self.camera = picamera.PiCamera()
        self.camera.resolution = (Mx,My)
        self.camera.rotation = roll
        self.num = 0
        self.path = path
        self.opath = opath
        self.format = format_
        self.filename = self.path+str(self.num)+self.format
        self.outfilename = self.opath+str(self.num)+self.format
        self.cascade = cv2.CascadeClassifier(cascadepath)
        self.height = target_height

    def getImage(self):
        self.num = self.num+1
        self.filename = self.path+str(self.num)+self.format
        self.outfilename = self.opath+str(self.num)+self.format
        self.camera.capture(self.filename)
        time.sleep(0.2)
        return True

    def detectTarget(self):
        img = cv2.imread(self.filename,cv2.IMREAD_GRAYSCALE)
        target = self.cascade.detectMultiScale(img)
        length = len(target)
        if length == 0:
            return False
        average = 0
        for rect in target:
            self.left_top = tuple(rect[0:2])
            self.right_bottom = tuple(rect[0:2]+rect[2:4])
            cv2.rectangle(img,self.left_top,self.right_bottom,(0,0,255),thickness=2)
            average += (self.left_top[0]+self.right_bottom[0])/2
        average /= len(target)
        self.average = average
        cv2.imwrite(self.outfilename,img)
        return average

    #def getRange(self):
    #    top_bottom = self.left_top[1]-self.right_bottom[1]
        
    def judge(self):
        if self.average>self.Mx/4:
            if self.average<self.Mx*3/4:
                return True
            return False
        else:
            return False
