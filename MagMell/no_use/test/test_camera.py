from DRV_Camera import Camera
import time

myCamera = Camera(Mx=1024,My=768)
for i in range(3):
	start = time.time()
	print('started.')
	myCamera.getImage()
	print('took a picture.')
	print('Number:',myCamera.num)
	print(myCamera.filename)
	var = myCamera.detectTarget()
	if var != False:
		print('EXEC_TIME:',time.time()-start)
		print('DET',var)
		print(myCamera.judge())
	else:
		print('EXEC_TIME:',time.time()-start)
		print('No target detected.')
