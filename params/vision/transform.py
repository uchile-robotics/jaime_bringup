#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

import roslib
import sys
sys.path.insert(0,"/home/matias/uchile_ws/ros/jaime/soft_ws/src/uchile_vision/yolov5/src/yolov5")
import rospy
import cv2
import numpy as np
import tf2_ros

from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from visualization_msgs.msg import Marker
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped

from detect import YoloV5


class distance_to_object_rviz:
    def __init__(self):
        self.sub_color = rospy.Subscriber("/camera/color/image_raw",Image,self.img_callback)
        self.sub_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.update_pinhole)
        self.pub_img = rospy.Publisher("/camera/color/image_detected", Image, queue_size=1)
        self.pub_marker = rospy.Publisher("/test_marker", Marker, queue_size=1)

        self.model = YoloV5(weights='/home/matias/uchile_ws/ros/jaime/soft_ws/src/uchile_vision/yolov5/yolov5_jp.pt') # poner aqui el path completo de los pesos
        self.names = self.model.names
        self.Boxes = None
        self.img = None
        self.br =  CvBridge()
        self.pinhole = None
        self.depth_img = None
        self.depth_raw = None
        self.marker = None

    def img_callback(self, msg):
        depth = rospy.wait_for_message("/camera/depth/image_raw", Image) #este topico cambia, para camara real: /camera/aligned_depth_to_color/image_raw
        self.detections(msg)
        self.process_depth(depth)

        box = self.Boxes.bounding_boxes[0]
        p,xmin,xmax,ymin,ymax = box.probability, box.xmin, box.xmax, box.ymin, box.ymax
        x_c, y_c = int((xmin.item()+xmax.item())//2), int((ymin.item()+ymax.item())//2)
        d = self.depth_raw[x_c, y_c]/100
        v = self.pinhole.projectPixelTo3dRay((x_c, y_c))
        p = np.multiply(d,v)
        print(v, "l",p, "l", d)
        tf = self.get_relative_coordinate("map", "camera_link")
        tf_x, tf_y, tf_z = tf.translation.x, tf.translation.y, tf.translation.z
        tf_array = np.array([tf_x, tf_y, tf_z])
        obj_pos = np.array([p[0] + tf_x, p[1] + tf_y, (p[2] + tf_z)/10])
        print("objpos: ",obj_pos)
        #print(obj_pos)
        self.make_marker(obj_pos)



    def update_pinhole(self, msg):
        self.pinhole = PinholeCameraModel()
        self.pinhole.fromCameraInfo(msg)
    
    def process_depth(self, msg):
        try:
            msg.encoding='mono16'
            cv_image_d = self.br.imgmsg_to_cv2(msg, "mono16") 
        except CvBridgeError as e:
            print(e)
        img_n = cv2.normalize(src=cv_image_d, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        im_color = cv2.cvtColor(img_n, cv2.COLOR_GRAY2BGR)
        self.depth_img = im_color
        self.depth_raw = cv_image_d


    def detections(self, msg):
        self.img = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.model.detect(self.img)
        boxes = BoundingBoxes()
        if not detections == []:
            for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                _x = int(x1.item()) #round(x2.item() - x1.item())/2 + x1.item()
                _y = int(y1.item())
                _w = int(round(x2.item() - x1.item()))
                _h = int(round(y2.item() - y1.item()))
                label = str(self.names[int(i_label.item())])
                prob = cls_conf.item()
                # creando el mensaje
                box = BoundingBox()
                box.probability = prob
                box.xmin = x1
                box.ymin = y1
                box.xmax = x2
                box.ymax = y2
                box.id = i_label.item()
                box.Class = label
                #la ponemos en la lista de bounding boxes
                boxes.bounding_boxes.append(box)
                
                self.img = cv2.rectangle(self.img, (_x, _y), (_x +_w, _y + _h), (0,255,0), 2)
                self.img = cv2.putText(self.img, label, (_x,_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                #print("detecte un",label, " con confianza de ", cls_conf.item())
        self.Boxes = boxes
        self.pub_img.publish(self.br.cv2_to_imgmsg(self.img, "bgr8"))

    def get_relative_coordinate(self, parent, child):
        u"""相対座標を取得する関数
        引数：
            parent (str): 親の座標系
            child (str): 子の座標系
        """

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        trans = TransformStamped()
        while not rospy.is_shutdown():
            try:
                # 4秒待機して各tfが存在すれば相対関係をセット
                trans = tfBuffer.lookup_transform(parent, child,
                                                rospy.Time().now(),
                                                rospy.Duration(4.0))
                break
            except (tf2_ros.ExtrapolationException):
                pass

        return trans.transform
    
    def make_marker(self, pos):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.ns = "object"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = pos[0]
        self.marker.pose.position.y = pos[1]
        self.marker.pose.position.z = pos[2]
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.lifetime = rospy.Duration(0)
        self.pub_marker.publish(self.marker)

def main():
    dist = distance_to_object_rviz()
    rospy.init_node("dist_rviz", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Apagando modulo de deteccion de imagenes")

if __name__=="__main__":
    main()
