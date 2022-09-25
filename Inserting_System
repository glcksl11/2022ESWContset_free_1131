import cv2
import numpy as np
import sys
import serial
import time
ser = serial.Serial('/dev/ttyAMA0',baudrate = 115200,parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS,timeout=1)

if ser.isOpen() ==False:
    ser.open()
    
def fit_rotated_ellipse(data): #least square method 를 이용한 ellipse fitting
    xs = data[:, 0].reshape(-1, 1)
    ys = data[:, 1].reshape(-1, 1)

    J = np.mat(np.hstack((xs * ys, ys ** 2, xs, ys, np.ones_like(xs, dtype=np.float))))
    Y = np.mat(-1 * xs ** 2)
    P = (J.T * J).I * J.T * Y

    a = 1.0;
    b = P[0, 0];
    c = P[1, 0];
    d = P[2, 0];
    e = P[3, 0];
    f = P[4, 0];
    theta = 0.5 * np.arctan(b / (a - c))

    cx = (2 * c * d - b * e) / (b ** 2 - 4 * a * c)
    cy = (2 * a * e - b * d) / (b ** 2 - 4 * a * c)

    cu = a * cx ** 2 + b * cx * cy + c * cy ** 2 - f
    w = np.sqrt(cu / (a * np.cos(theta) ** 2 + b * np.cos(theta) * np.sin(theta) + c * np.sin(theta) ** 2))
    h = np.sqrt(cu / (a * np.sin(theta) ** 2 - b * np.cos(theta) * np.sin(theta) + c * np.cos(theta) ** 2))
    ellipse_model = lambda x, y: a * x ** 2 + b * x * y + c * y ** 2 + d * x + e * y + f

    #error_sum = np.sum([ellipse_model(x, y) for x, y in data])
    #print('fitting error = %.3f' % (error_sum))

    return (cx, cy, w, h, theta)

def nothing(x):
    pass

cv2.namedWindow("Canny Edge") #canny_edge trackbar
cv2.createTrackbar('low threshold', 'Canny Edge', 0, 255, nothing)
cv2.createTrackbar('high threshold', 'Canny Edge', 0, 255, nothing)

cv2.setTrackbarPos('low threshold', 'Canny Edge', 50)
cv2.setTrackbarPos('high threshold', 'Canny Edge', 150)

try :
    cap = cv2.VideoCapture(cv2.CAP_V4L2)
except Exception as e:
    print(e)
    sys.exit()
#cap.set(3,320)
#cap.set(4,240)

flag = 1
px = 0
py = 0
pw = 0
ph = 0
pt = 0
pa=  1
send_words = ""
send_x = 0
send_y = 0
send_zero = 0
plz1 = '('
plz2 = ','
plz3 = ')'
plz4 = '['
plz5 = ']'
insert_y = 700
while True:    
    if( cv2.waitKey(100) & 0XFF == 49):
        flag = 0
    elif( cv2.waitKey(100) & 0xFF == 8):
        send_words = input("give me words : ")
        print(send_words)
        ser.write(send_words.encode('utf-8'))
        send_words = ""
    elif( cv2.waitKey(100) & 0xFF == 50):
        ser.write(plz4.encode('utf-8'))
        ser.write(str(insert_y).encode('utf-8'))
        ser.write(plz5.encode('utf-8'))
        print("insert_y success")
    elif( cv2.waitKey(100) & 0xFF == 51):
        ser.write(plz1.encode('utf-8'))
        ser.write(str(send_x).encode('utf-8'))
        ser.write(plz2.encode('utf-8'))
        ser.write(str(send_zero).encode('utf-8'))
        ser.write(plz3.encode('utf-8'))
        print("all pass")
        time.sleep(5)
        ser.write(plz4.encode('utf-8'))
        ser.write(str(send_zero).encode('utf-8'))
        ser.write(plz5.encode('utf-8'))
    elif( cv2.waitKey(100) & 0xFF == 52):
        ser.write(plz1.encode('utf-8'))
        ser.write(str(send_zero).encode('utf-8'))
        ser.write(plz2.encode('utf-8'))
        ser.write(str(send_zero).encode('utf-8'))
        ser.write(plz3.encode('utf-8'))
        print("initiate 0 0 0")
        time.sleep(2)
        ser.write(plz4.encode('utf-8'))
        ser.write(str(send_zero).encode('utf-8'))
        ser.write(plz5.encode('utf-8'))
        
        
       
    low = cv2.getTrackbarPos('low threshold', 'Canny Edge')
    high = cv2.getTrackbarPos('high threshold', 'Canny Edge')

    ret, frame = cap.read()

    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_canny = cv2.Canny(img_gray,low,high)
    contours, _ = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for con in contours:
        area = cv2.contourArea(con)
        if (len(con) > 10 and area > 1000):
            # fit ellipse
            sample = np.random.choice(len(con), 15, replace=True) #임의의 10개 sample data index 추출
            cx, cy, w, h, theta = fit_rotated_ellipse(con[sample].reshape(-1, 2)) #컨투어 좌표들중 samle index만 사용하여 ellipse fitting
            if(w<=h) : aspect_ratio = w/h
            else: aspect_ratio =h/w
            if(aspect_ratio <= 1 and aspect_ratio > 0.9 and flag ==0): # 너무 찌그러진 타원은 제외
                px = cx
                py = 480-cy
                #print(px,py)
                #pts1 = np.float32([[130,147],[557,156],[133,314]])
                pts1 = np.float32([[213,180],[453,170],[218,347]])
                pts2 = np.float32([[0,0],[235,0],[0,170]])
                M = cv2.getAffineTransform(pts1,pts2) #getPerspectiveTransform
                #cx = (int)(cx)
                #cy = (int)(cy)
                point = np.float32([px,py,1])
                #print(point)
                send_point = np.dot(M,point.T)
                #print(send_point)
                send_x = (int)(send_point[0])
                send_y = (int)(send_point[1])
                send_x = send_x + 4
                send_y = send_y - 5
                print(send_x,send_y)
                ser.write(plz1.encode('utf-8'))
                #ser.write(bytes(str(send_x),encoding='ascii'))
                ser.write(str(send_x).encode('utf-8'))
                ser.write(plz2.encode('utf-8'))
                ser.write(str(send_y).encode('utf-8'))
                ser.write(plz3.encode('utf-8'))
                pw = w
                ph = h
                pt = theta
                pa = aspect_ratio
                flag = 1
    cv2.ellipse(frame, (int(px), int(480-py)), (int(np.nan_to_num(pw)), int(np.nan_to_num(ph))), pt * 180.0 / np.pi, 0.0,
                360.0, (255, 0, 0), 2)
    cv2.putText(frame, "center" + str(int(px)) + "," + str(int(480-py)) + "," + str(pa), (30, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 50, 170), 2)
    cv2.putText(frame, send_words, (60, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (170, 50, 50), 2)
    cv2.imshow('Canny Edge',img_canny)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break



