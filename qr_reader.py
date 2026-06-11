#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar

class QRReaderNode:
    def __init__(self):
        rospy.init_node('qr_reader_node')
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Senin sistemindeki gerçek kamera konusu ve dogru fonksiyon eslesmesi:
        rospy.Subscriber("/raspicam_node/image/mouse_click", Image, self.camera_callback)
        
        # Görev yöneticisinin çagirdigi servis:
        self.qr_service = rospy.Service('/decode_qr', Trigger, self.handle_decode_qr)
        rospy.loginfo("QR Reader Servisi Başarıyla Başlatıldı. Tetik bekleniyor...")

    def camera_callback(self, msg):
        # Kameradan gelen görüntüyü sürekli günceller
        self.latest_image = msg

    def handle_decode_qr(self, req):
        res = TriggerResponse()
        
        if self.latest_image is None:
            res.success = False
            res.message = "HATA: Kameradan henüz görüntü alınamadı!"
            return res
            
        try:
            # ROS görüntüsünü OpenCV formatına çeviriyoruz
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            # QR kodlari tara
            qr_codes = pyzbar.decode(cv_image)
            
            if qr_codes:
                qr_data = qr_codes[0].data.decode('utf-8')
                res.success = True
                res.message = qr_data
            else:
                res.success = False
                res.message = "Görünürde QR kod bulunamadi."
                
        except Exception as e:
            res.success = False
            res.message = f"Görüntü isleme hatasi: {str(e)}"
            
        return res

if __name__ == '__main__':
    try:
        node = QRReaderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
