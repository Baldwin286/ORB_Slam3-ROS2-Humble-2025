import cv2
import numpy as np
import os

chessboard_size = (9, 6)  

obj_points = []  
img_points = []  

obj_point = np.zeros((np.prod(chessboard_size), 3), dtype=np.float32)
obj_point[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)


image_folder = '/home/phamthanhbien/Pictures/'


images = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]


for image_file in images:
    img = cv2.imread(os.path.join(image_folder, image_file))
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret:
        img_points.append(corners)
        
        obj_points.append(obj_point)
        
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        
        cv2.imshow('Chessboard', img)
        cv2.waitKey(500)  

cv2.destroyAllWindows()

if len(obj_points) > 0 and len(img_points) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    
    if ret:
        print("camera calibration successful.")
        print(f"Matrix of camera (mtx):\n{mtx}")
        print(f"Vector defomation (dist):\n{dist}")
        
        fx = mtx[0, 0]  
        fy = mtx[1, 1]  
        cx = mtx[0, 2]  
        cy = mtx[1, 2]  
        
        k1, k2, p1, p2, k3 = dist.flatten()
        
        print(f"Camera.fx: {fx}")
        print(f"Camera.fy: {fy}")
        print(f"Camera.cx: {cx}")
        print(f"Camera.cy: {cy}")
        print(f"Camera.k1: {k1}")
        print(f"Camera.k2: {k2}")
        print(f"Camera.p1: {p1}")
        print(f"Camera.p2: {p2}")
        print(f"Camera.k3: {k3}")
    else:
        print("Error when calibrating camera.")
else:
    print("Not enough data for camera calibration.")
