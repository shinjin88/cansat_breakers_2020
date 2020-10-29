import cv2

print(cv2.__version__)
filename = 'IMG_0649.jpg'

c_path = 'cascade_traffic_cone.xml'
cascade = cv2.CascadeClassifier(c_path)
img = cv2.imread(filename)

rect = cascade.detectMultiScale(img)
color = (255,0,0)

if len(rect) > 0:
	for i in rect:
		left_top = tuple(i[0:2])
		right_bottom = tuple(i[0:2]+i[2:4])
		print(left_top[0])
		print(right_bottom[0])
		cv2.rectangle(img,left_top,right_bottom,color,thickness=2)
	cv2.imwrite('test~.jpg',img)
else:
	print('no target detected.')

