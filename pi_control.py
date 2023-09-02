import RPi.GPIO as GPIO
import time
import threading
import cv2
import numpy as np

core_x=320
core_y=240
area=0
#init
angle_high=90
angle_lever=90
#record
angle1=90
angle2=90
# 设置舵机引脚
servo1_pin = 7
servo2_pin = 6

# 初始化 GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

# 创建 PWM 对象
servo1_pwm = GPIO.PWM(servo1_pin, 50)
servo2_pwm = GPIO.PWM(servo2_pin, 50)

# 启动 PWM
servo1_pwm.start(0)
servo2_pwm.start(0)

# 定义要追踪的颜色范围（以HSV格式表示）
colors = [
    {
        'name': 'red',
        'lower': np.array([153, 120, 70]),
        'upper': np.array([185, 255, 255]),
        'color': (0, 0, 255)
    },
    {
        'name': 'green',
        'lower': np.array([37, 120, 70]),
        'upper': np.array([80, 255, 255]),
        'color': (0, 255, 0)
    },
    {
        'name': 'orange',
        'lower': np.array([0, 120, 70]),
        'upper': np.array([25, 255, 255]),
        'color': (0, 125, 125)
    }
]

# 初始化摄像头
cap = cv2.VideoCapture(0)
loc = input("color:")

KP=0.1
KI=0.15
KD=0.1

class PID():
	def __init__(self, core_X, core_Y):
		self.X = core_X
		self.Y = core_Y
		self.goal_x = 320
		self.goal_y = 240
		self.now_err_x = 0
		self.pre_err_x = 0
		self.last_err_x = 0
		self.output_X = 0
		self.now_err_y = 0
		self.pre_err_y = 0
		self.last_err_y = 0
		self.output_Y = 0
	def cmd_pid_x(self):
		global KP
		global KI
		global KD
		if(self.goal_x==self.X):
			return 0
		self.now_err_x=self.goal_x - self.X
		self.output_X=KP*(self.now_err_x-self.last_err_x) + KI*self.now_err_x + KD*(self.now_err_x - self.last_err_x +self.pre_err_x)
		self.pre_err_x=self.last_err_x
		self.last_err_x=self.now_err_x
		#print(self.output_X)
		return  self.output_X
	def cmd_pid_y(self):
		global KP
		global KI
		global KD
		if(self.goal_y==self.Y):
			return 0
		self.now_err_y=self.goal_y - self.Y
		self.output_Y=KP*(self.now_err_y-self.last_err_y) + KI*self.now_err_y + KD*(self.now_err_y - self.last_err_y +self.pre_err_y)
		self.pre_err_y=self.last_err_y
		self.last_err_y=self.now_err_y
        #print(self.output_Y)
		return  self.output_Y
def set_servo_angle(pwm, angle,old_angle):
	if angle==old_angle:
		pass
	else:
		duty = (angle / 180) * 10 + 2.5  # 将角度转换为占空比
		pwm.ChangeDutyCycle(duty)
		time.sleep(0.4)
		pwm.ChangeDutyCycle(0)

def control_servo1():
	global core_x
	global core_y
	global area
	global angle_high
	global angle1
	try:
		while True:
			if angle_high > 180:
				angle_high = 180
			if (angle_high < 0):
				angle_high = 0
			if(area>=500):
				if(core_y>=300 or core_y <=200):
					my_Pid = PID(core_x, core_y)
					angle_high = angle_high + int(my_Pid.cmd_pid_y()) // 5
					set_servo_angle(servo1_pwm, angle_high,angle1)
					angle1=angle_high
			else:
				pass
	except KeyboardInterrupt:
			pass

def control_servo2():
	global core_x
	global core_y
	global area
	global angle_lever
	global angle2
	find=0
	try:
		while True:
			if angle_lever > 180:
				angle_lever = 180
			if (angle_lever < 0):
				angle_lever = 0
			if(area>=500):
				if (core_x >=400  or core_x <= 200):
					my_Pid = PID(core_x, core_y)
					angle_lever = angle_lever + int(my_Pid.cmd_pid_x()) // 5
					set_servo_angle(servo2_pwm, angle_lever,angle2)
					angle2=angle_lever
			else:
				for i in range(11):
					if(area>=500):
						my_Pid = PID(core_x, core_y)
						angle_lever = angle_lever + int(my_Pid.cmd_pid_x()) // 5
						set_servo_angle(servo2_pwm, angle_lever, angle2)
						angle2 = angle_lever
						break
					angle_lever+=(5*i)
					set_servo_angle(servo2_pwm, angle_lever, angle2)
					angle2 = angle_lever
					if(i==10):
						angle_lever=90
						break
				i=0
				for i in range(11):
					if(area>=500):
						my_Pid = PID(core_x, core_y)
						angle_lever = angle_lever + int(my_Pid.cmd_pid_x()) // 5
						set_servo_angle(servo2_pwm, angle_lever, angle2)
						angle2 = angle_lever
						break
					angle_lever-=(5*i)
					set_servo_angle(servo2_pwm, angle_lever, angle2)
					angle2 = angle_lever

	except KeyboardInterrupt:
		pass
        
def camero():
	global area
	global core_x
	global core_y
	for color in colors:
		# 根据颜色范围创建掩膜
		if color['name'] == loc:
			break
	try:
		while True:
			# 读取摄像头画面
			ret, frame = cap.read()
			# 转换颜色空间为HSV
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			# 创建掩膜，根据设定的颜色范围来提取目标颜色
			mask = cv2.inRange(hsv, color['lower'], color['upper'])
			# 对掩膜进行形态学操作，去除噪声
			kernel = np.ones((5, 5), np.uint8)
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
			# 寻找轮廓
			contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			# 绘制矩形框
			for contour in contours:
				area = cv2.contourArea(contour)
				#update(area)
				if area > 800:
					find=1
					x, y, w, h = cv2.boundingRect(contour)
					cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
					core_x=x+w//2
					core_y=y+h//2
					#描绘中心点
					cv2.circle(frame, (int(core_x), int(core_y)), 5, color['color'], -1) 
					cv2.putText(frame, 'x,y:' + str(core_x) +'  '+ str(core_y), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			# 显示结果
			#cv2.imshow('mask',mask)
			cv2.imshow('Color Tracking', frame)

			# 按下'q'键退出
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		# 释放摄像头资源
		cap.release()
		cv2.destroyAllWindows()
	except KeyboardInterrupt:
		pass


try:
    # 创建两个线程分别控制舵机
    thread1 = threading.Thread(target=control_servo1)
    thread2 = threading.Thread(target=control_servo2)
    thread3 = threading.Thread(target=camero)

    # 启动线程
    thread1.start()
    thread2.start()
    thread3.start()

    # 等待线程结束
    thread1.join()
    thread2.join()
    thread3.join()
    

except KeyboardInterrupt:
    pass

# 停止 PWM
servo1_pwm.stop()
servo2_pwm.stop()

# 清理 GPIO
GPIO.cleanup()
