#Подключаем необходимые библиотеки и функции
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from collections import  deque

rospy.init_node('flight')
bridge = CvBridge()

# Устанавливаем прокси сервисы для функций
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect, persistent=True)

# Задаем стандартные значия и ограничения очереди списка
mybuffer = 1024  
pts = deque(maxlen=mybuffer)
center = None

def image_callback(data):
    # Делаем глобальной переменную Green
    global green
    # Создаем переменную в которой хранится картина с камеры Raspberry PI
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    # Переводим картинку камеры в другую цветовую палитру (HSV)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Создаем переменную идентификации нахождения нужного цвета
    green = cv2.inRange(hsv, (50, 210, 150), (70, 255, 215))

    # Производим фильтрацию данных
    mask = cv2.erode(green, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)  

    # Находим границы идентифицированного цветного объекта
    _, cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Отображаем найденную границу
    cv2.drawContours( cv_image, cnts, -1, (255,0,0), 3, cv2.LINE_AA, hierarchy, 1 )

    #Находим центр обнаруженного объекта
    if len(cnts) > 0:    
        c = max(cnts, key = cv2.contourArea)    
        ((x, y), radius) = cv2.minEnclosingCircle(c)    
        M = cv2.moments(c)   
        center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        pts.appendleft(center) 
    # Рисуем траекторию движения используя координаты настоящей и предыдущей точки объекта
    for i in xrange(1, len(pts)):  
        if pts[i - 1] is None or pts[i] is None:  
            continue   

        cv2.line(cv_image, pts[i - 1], pts[i], (0, 0, 255), 4)
    # Публикуем картинку с рисующейся траекторией в топик, для того чтобы иметь возможность наблюдать все в реальном времени
    image_pub = rospy.Publisher('~debug', Image, queue_size=1)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

# Подписываемся на стандартный топик камеры для корректной работы системы слежения
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()
