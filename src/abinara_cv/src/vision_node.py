#!/usr/bin/env python3
import cv2, time, numpy as np, rospy
from abinara_cv.msg import VisionInfo

def detect_shape(cnt):
    approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
    sides = len(approx)
    if sides == 3:
        return "Segitiga"
    elif sides == 4:
        return "Persegi"
    else:
        return "Lingkaran"

def main():
    rospy.init_node("vision_publisher")
    pub = rospy.Publisher("/vision/info", VisionInfo, queue_size=10)

    cap = cv2.VideoCapture(0)
    cap.set(3, 640) 
    cap.set(4, 480)  

    prev_t = time.time()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Camera not found!")
            break

        frame = cv2.flip(frame, 1) 

        
        now = time.time()
        fps = 1.0 / (now - prev_t)
        prev_t = now

        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        
        mask_or = cv2.inRange(hsv, np.array([5,120,150]), np.array([20,255,255]))
        
        mask_wh = cv2.inRange(hsv, np.array([0,0,200]), np.array([180,40,255]))

        
        kernel = np.ones((5,5),np.uint8)
        mask_or = cv2.morphologyEx(mask_or, cv2.MORPH_OPEN, kernel)
        mask_wh = cv2.morphologyEx(mask_wh, cv2.MORPH_OPEN, kernel)

        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        edges = cv2.Canny(blur,50,150)

        cnts,_ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        shape = ""
        color = ""

        for c in cnts:
            if cv2.contourArea(c) > 800:
                shape = detect_shape(c)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(frame, shape, (x,y-10), cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
                break

        
        if np.sum(mask_or) > 20000:
            color = "Orange"
        elif np.sum(mask_wh) > 20000:
            color = "Putih"

        cv2.putText(frame, f"FPS:{int(fps)}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(frame, f"Shape:{shape}", (10,55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(frame, f"Color:{color}", (10,85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,140,255), 2)

        
        msg = VisionInfo()
        msg.shape = shape
        msg.color = color
        msg.fps = float(fps)
        pub.publish(msg)

        cv2.imshow("Vision Node", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
