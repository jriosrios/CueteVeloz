import cv2

cap = cv2.VideoCapture('vid.mp4')

if not cap.isOpened():
    print("Error al abrir el video")
    
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = gray[300:500,:]
        median = cv2.medianBlur(frame, 5)
        canny = cv2.Canny(median, 100, 200)
        
        cv2.imshow('Video MP4', canny)
        
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
