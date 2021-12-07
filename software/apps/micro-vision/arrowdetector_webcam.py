import cv2
import numpy as np
import serial
import time
import queue
import threading

##test_image = cv2.imread("C:/Users/joshu/Documents/EE332/Project/test.png")
##test_image2 = cv2.imread("C:/Users/joshu/Documents/EE332/Project/test_rot1.png")
##test_image3 = cv2.imread("C:/Users/joshu/Documents/EE332/Project/test_rot2.png")
##test_image4 = cv2.imread("C:/Users/joshu/Documents/EE332/Project/test_rot3.png")
##test_image5 = cv2.imread("C:/Users/joshu/Documents/EE332/Project/test_rot4.png")
##test2_img = cv2.imread("C:/Users/joshu/Documents/EE332/MP6/test.bmp")

class BufferlessVideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except Queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()


video_capture = BufferlessVideoCapture(0)
display_tests = False


def hough_transform(img):
    quant_rho = 500
    rho_quant_mul = np.sqrt(img.shape[0]**2 + img.shape[1]**2) / quant_rho#rho is between 0 and diagonal of image
    quant_theta = 500
    theta_quant_mul = np.pi / quant_theta;
    theta_rho = get_parameter_intersects(img, quant_rho, quant_theta, rho_quant_mul, theta_quant_mul)
    
    if display_tests:
        intersections = np.zeros(theta_rho.shape)
        intersections[theta_rho>0] = 255
        cv2.imshow("Intersections", intersections)

    res = draw_lines(img, theta_rho, theta_quant_mul, rho_quant_mul)
    
    if display_tests:
        cv2.imshow("Hough", res)
    return res

def draw_lines(img, theta_rho, theta_quant_mul, rho_quant_mul):
    res = np.zeros(img.shape)
    for t in range(theta_rho.shape[0]):
        for r in range(theta_rho.shape[1]):
            if theta_rho[t][r] > 0:
                theta = (t * theta_quant_mul) - np.pi/4
                rho = r * rho_quant_mul
                #print(t)
                for i in range(res.shape[0]):
                    y = (rho - (i * np.cos(theta))) / np.sin(theta)
                    #print(y)
                    if y >= 0 and y < res.shape[1]:
                        res[i][int(y)] = 255
                        #print(y)
                    #else:
                        #print(rho)
    return res

def get_parameter_intersects(img, quant_rho, quant_theta, rho_quant_mul, theta_quant_mul):
    E = cv2.Canny(img, 100, 200)
    if display_tests:
        cv2.imshow("Canny", E)
    theta_rho = np.zeros((quant_theta, quant_rho))

    for i in range(E.shape[0]):
        for j in range(E.shape[1]):
            if E[i][j] > 0:
                for theta_q in range(quant_rho):
                    theta = (theta_q * theta_quant_mul) - np.pi/4
                    rho = (i * np.cos(theta)) + (j * np.sin(theta))
                    theta_rho[theta_q][int(rho / rho_quant_mul)] += 1
    if display_tests:
        cv2.imshow("Parameter space", theta_rho / 10)
    
    thresh = 20
    for t in range(theta_rho.shape[0]):
        for r in range(theta_rho.shape[1]):
            if theta_rho[t][r] < thresh:
                theta_rho[t][r] = 0
            else:
                for i in range(int(-((4/25)*quant_theta)), int(((4/25)*quant_theta))+1):
                    if t + i < 0:
                        i = theta_rho.shape[0] - i
                    if t + i >= theta_rho.shape[0]:
                        i = i - theta_rho.shape[0]
                    for j in range(int(-((4/25)*quant_rho)), int(((4/25)*quant_rho))+1):
                        oob_lr = t + i < 0 or t + i >= theta_rho.shape[0]
                        oob_ud = r + j < 0 or r + j >= theta_rho.shape[1]
                        if (not (oob_lr or oob_ud)) and (not ((i == j) and i == 0)) and theta_rho[t+i][r+j] <= theta_rho[t][r]:
                            theta_rho[t+i][r+j] = 0
    return theta_rho

def get_arrow_dir(img):
    angle = 45 * 2*np.pi/360
    color_lower = np.array([170/2, 100, 100])
    color_upper = np.array([210/2, 255, 255])
    #cv2.imshow('Image', img)
    #cv2.imshow('Mask', cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), color_lower, color_upper))
    c = cv2.Canny(cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), color_lower, color_upper), 100, 200)
    #cv2.imshow('Canny', c)
    #cv2.imshow('Webcam', img)
    intersects = cv2.HoughLines(c,1,np.pi/180, 50)
    arrows = []
    #print(intersects)

    ttol = np.pi / 50
    if intersects is None:
        return 0
    for i in range(intersects.shape[0]):
        bf = False
        for j in range(i+1, intersects.shape[0]):
            if j >= intersects.shape[0]:
                continue
            if abs(abs(intersects[i][0][1] - intersects[j][0][1]) - angle) <= ttol:
                arrows.append([intersects[i][0][1], intersects[j][0][1], intersects[i][0][0], intersects[j][0][0]])
                bf = True
                break
        if bf:
            break

    if len(arrows) < 1:
        return 0
    [t1, t2, r1, r2] = arrows[0]
    dir_deg = np.rad2deg((((t1 + t2) / 2)))
    
    if dir_deg > 90:
        dir_deg = dir_deg - 180
    if dir_deg < -90:
        print(dir_deg)
        dir_deg = 180 + dir_deg
    if display_tests:
        cv2.imshow('Arrows', draw_lines(img, arrows, theta_quant_mul, rho_quant_mul))
        cv2.imshow('Dir', draw_lines(img, dir_arr, theta_quant_mul, rho_quant_mul))
    if abs(dir_deg) > 80:
        dir_deg = 0
    return dir_deg

def get_microbit_line():
    line = ser.readline()
    line_str = line.decode('utf-8')
    print("Microbit response: " + line_str, end = "")
    return line_str

port = '/dev/ttyACM0'
ser = serial.Serial(port, 38400)
print(ser.name)
#ser.write(b'Ready')
prev_arr_dir = [0, 0, 0, 0]#, 0, 0, 0, 0, 0, 0]
diff_tol = 10
print('Starting detection loop')
while True:
    frame = video_capture.read()
    arr_dir = get_arrow_dir(frame)
    prev_arr_dir.pop(0)
    prev_arr_dir.append(arr_dir)
    valid = True
    avg = sum(prev_arr_dir)/len(prev_arr_dir)
    for direction in prev_arr_dir:
        if abs(direction - avg) > diff_tol:
            valid = False

    if valid and abs(avg) > diff_tol:
        print(avg)
        avg_unsigned = avg + 256 if avg < 0 else abs(avg)
        ser.write(int(avg_unsigned).to_bytes(1, 'big'))
        while not get_microbit_line().startswith("done turn"):
            pass
    else:
        pass
        #print(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
video_capture.release()
