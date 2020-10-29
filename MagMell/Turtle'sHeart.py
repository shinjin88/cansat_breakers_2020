#!/usr/bin/python3
from UtopioSphere import RunBack as LibRunBack
import json
import argparse
import subprocess
import time

file = open('./config.json','r')
conf = json.load(file)
#Goal Ref
GOAL_LAT = conf['config']['control']['default']['goal_lat']
GOAL_LNG = conf['config']['control']['default']['goal_lng']

#Range for modechange
mrange = conf['config']['control']['default']['distance_mode-change']
#Range for Goal
grange = conf['config']['control']['default']['distance_goal']
file.close()





def RunBack(lat,lng,Range,Goal_Range,skipdrop=False,skippurge=False,disable_drill=False,disable_camera=False,print_to_console=False):
    DRV = LibRunBack(lat,lng,Goal_Range,print_to_console)
    DRV.loadCalibration(False,False,True)
    if skipdrop == False:
        DRV.waitDrop()
        time.sleep(10)
    if skippurge == False:
        DRV.purgeByServo()
    DRV.getLatLngv2(True)
    DRV.moveForward(1)
    if disable_drill == False:
        DRV.Drill(10)
    DRV.getLatLngv2(True)
    DRV.face2Goal()
    while True:
        if DRV.DistancetoG['distance'] < Goal_Range:
            DRV.getLatLngv2(True)
            DRV.transmit('Goal:'+str(DRV.DistancetoG['distance']))
            return
        elif DRV.DistancetoG['distance'] > Range:
            DRV.moveForward(DRV.updateMoveTime(DRV.DistancetoG['distance']))
            DRV.getLatLngv2(False)
            DRV.face2Goal()
            continue
        elif (DRV.DistancetoG['distance'] < Range) and (disable_camera == False):
            DRV.detectTargetfromPicture()
            DRV.getLatLngv2(True)

def Manual(lat,lng,Goal_Range):
    DRV = LibRunBack(lat,lng,Goal_Range,True)
    DRV.getLatLngv2()
    print('Mode:Manual')
    while True:
        com = input('COMMAND>>')
        com_list = com.split() 
        leng = len(com_list)
        if leng == 0:
            print('Usage:\nmve,mvem,trn,esc\ngps,cam,clb,sts\nhelp,exit')
            continue
        elif leng > 2:
            ('Error. invalid command.' )
        elif leng == 1:
            com_list.append(None)
        if (com_list[0] == 'mve') or (com_list[0] == 'moveForward'):
            if com_list[1] == None:
                DRV.getLatLngv2()
                DRV.moveForward(DRV.updateMoveTime(DRV.DistancetoG['distanceS2G']))
            else:
                DRV.moveForward(com_list[1])
        elif (com_list[0] == 'mvem') or (com_list[0] == 'moveForward-m'):
            if com_list[1] == None:
                DRV.getLatLngv2()
                DRV.moveForward(DRV.updateMoveTime(DRV.DistancetoG['distanceS2G']))
            else:
                DRV.moveForward(DRV.updateMoveTime(com_list[1]))
        elif (com_list[0] == 'trn') or (com_list[0] == 'Turn') or (com_list[0] == 'Face2'):
            if com_list[1] == None:
                DRV.getLatLngv2()
                DRV.face2Goal()
            else:
                DRV.face2(com_list[1])
        elif (com_list[0] == 'esc') or (com_list[0] == 'escapeStack'):
            DRV.escapeFromStack()
        elif (com_list[0] == 'gps') or (com_list[0] == 'getLatLng'):
            str(DRV.getLatLngv2())
        elif (com_list[0] == 'cam') or (com_list[0] == 'detectTarget'):
            DRV.DRV_Camera.getImage()
            print(DRV.DRV_Camera.detectTarget())
        elif (com_list[0] == 'sts') or (com_list[0] == 'showStatus'):
            print(str(vars(DRV)))
        elif (com_list[0] == 'clb') or (com_list[0] == 'calibration'):
            DRV.calibrateAG()
            DRV.calibrateM()
            DRV.loadCalibration()
        elif (com_list[0] == 'ext') or (com_list[0] == 'exit'):
            return
        elif (com_list[0] == 'h') or (com_list[0] == 'help'):
            print('Usage:\nmve,mvem,trn,esc\ngps,cam,clb,sts\nhelp,exit')
        elif (com_list[0] == '999'):
            cmd = f"sl -a -l -F"
            subprocess.run(cmd,shell=True,text=True)        
        else:
            print('Error. invalid command.' )


if __name__ ==  '__main__':
    parser = argparse.ArgumentParser(description='ランバック制御用プログラム　2020-9')
    parser.add_argument('--latitude',type=float,help='緯度の指定、なければデフォルトの値を利用する')
    parser.add_argument('--longitude',type=float,help='経度の指定、なければデフォルトの値を利用する')
    parser.add_argument('-m','--distance-mode',type=int,help='誘導方式が変わる距離の指定、なければデフォルトの値を利用する')
    parser.add_argument('-g','--distance-goal',type=int,help='ゴール判定の距離の指定、なければデフォルトの値を利用する')
    parser.add_argument('-c','--console',action='store_true',help='通信の出力を標準出力へ流す')
    parser.add_argument('-M','--manual',action='store_true',help='手動での制御に切り替える')
    parser.add_argument('-d','--disable-drill',action='store_true',help='ドリルの工程をスキップする')
    parser.add_argument('-r','--disable-drop',action='store_true',help='投下検知をしない')
    parser.add_argument('-p','--disable-purge',action='store_true',help='パラシュートのパージ工程をスキップする')
    parser.add_argument('-C','--disable-camera',action='store_true',help='カメラによる誘導を無効にする')
    args = parser.parse_args()
    if args.latitude != None:
        lat = args.latitude
    else:
        lat = GOAL_LAT
    if args.longitude != None:
        lng = args.longitude
    else:
        lng = GOAL_LNG
    if args.distance_mode != None:
        mrange = args.distance_mode
    if args.distance_goal != None:
        grange = args.distance_goal
    if args.manual == True:
        Manual(lat,lng,grange)
    else:
        RunBack(lat,lng,mrange,grange,args.disable_drop,args.disable_purge,args.disable_drill,args.disable_camera,args.console)
