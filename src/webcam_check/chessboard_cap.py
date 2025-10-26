import cv2
import numpy as np
import os

chessboard_size = (9, 6)

save_folder = '/home/phamthanhbien/Pictures/'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("camera not found")
    exit()

print("Press 'c' button to cap image. press 'q' button to exit.")

image_counter = 1

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can not read frame from camera")
        break

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('c'):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.equalizeHist(gray)

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)

            filename = os.path.join(save_folder, f"chessboard_image_{image_counter}.jpg")
            
            cv2.imwrite(filename, frame)
            print(f"Image {filename} has been saved.")
            
            image_counter += 1
        else:
            print("No chessboard found, try again.")

    elif cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break

cap.release()
cv2.destroyAllWindows()

