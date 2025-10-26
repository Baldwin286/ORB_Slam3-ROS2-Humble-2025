import cv2

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("not found webcam")
    exit()

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print(f"webcam resolution: {width}x{height}")

cap.release()
