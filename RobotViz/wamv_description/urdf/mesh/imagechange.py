import cv2

img=cv2.imread("/home/ipac/catkin_ws/src/vrx/wamv_description/urdf/mesh/mark_n.jpg",cv2.IMREAD_COLOR) 
rows,cols,channels=img.shape
img_big = cv2.resize(img, (cols*10, rows*10), interpolation=cv2.INTER_AREA)
cv2.imwrite("/home/ipac/catkin_ws/src/vrx/wamv_description/urdf/mesh/mark_n_10.jpg",img_big)
cv2.imshow("big",img_big)
cv2.waitKey(0)