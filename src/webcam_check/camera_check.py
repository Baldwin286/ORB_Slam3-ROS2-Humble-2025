import cv2
import numpy as np
import os

mtx = np.array([[471.03019513, 0, 322.63369156],
                [0, 630.23894457, 260.5650966],
                [0, 0, 1]])

dist = np.array([[-0.33535209, -0.01436051, 0.00061654, 0.00324759, 0.24337811]])

image_folder = '/home/phamthanhbien/Pictures/'
image_files = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]  

for image_file in image_files:

    img = cv2.imread(os.path.join(image_folder, image_file))


    if img is None:
        print(f"Không thể đọc ảnh {image_file}")
        continue


    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    chessboard_size = (9, 6)
   
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)

        undistorted_image = cv2.undistort(img, mtx, dist)

        cv2.imshow(f"Chessboard with Corners - {image_file}", img)
        cv2.imshow(f"Undistorted Image - {image_file}", undistorted_image)

        cv2.waitKey(0)

    else:
        print(f"Không tìm thấy bàn cờ trong ảnh {image_file}.")

cv2.destroyAllWindows()
