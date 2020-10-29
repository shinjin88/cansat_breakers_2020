import sys
import subprocess

print('Mode:Manual')
while True:
    com = input('COMMAND>>')
    com_list = com.split() 
    leng = len(com_list)
    if leng == 0:
        print('Usage:')
        continue
    elif leng > 2:('Error. invalid command.' )
    elif leng == 1:
        com_list.append(None)
    if (com_list[0] == 'mve') or (com_list[0] == 'moveForward'):
        if com_list[1] == None:
            pass
        else:
            pass
    elif (com_list[0] == 'mvem') or (com_list[0] == 'moveForward-m'):
        if com_list[1] == None:
            pass
        else:
            pass
    elif (com_list[0] == 'trn') or (com_list[0] == 'Turn') or (com_list[0] == 'Face2'):
        if com_list[1] == None:
            pass
        else:
            pass
    elif (com_list[0] == 'esc') or (com_list[0] == 'escapeStack'):
        pass
    elif (com_list[0] == 'gps') or (com_list[0] == 'getLatLng'):
        pass
    elif (com_list[0] == 'cam') or (com_list[0] == 'detectTarget'):
        pass
    elif (com_list[0] == 'sts') or (com_list[0] == 'showStatus'):
        pass
    elif (com_list[0] == 'ext') or (com_list[0] == 'Exit'):
        sys.exit()
    elif (com_list[0] == 'h') or (com_list[0] == 'help'):
        pass
    elif (com_list[0] == '999'):
        cmd = f"sl -a -l -F"
        subprocess.run(cmd,shell=True,text=True)        
    else:
        print('Error. invalid command.' )

    print(str(com_list))