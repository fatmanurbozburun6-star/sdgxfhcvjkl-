#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class TurtleBotKontrol():
    def __init__(self):
        rospy.init_node("turtlebot3_pid_kontrol")
        
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaj = Twist()
        
        rospy.Subscriber("scan", LaserScan, self.lazerCallback)
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw", Image, self.kameraCallback)
        
        self.min_on = 0.0
        self.min_sol = 0.0
        self.min_sag = 0.0
        self.min_sol_arka = 0.0  # YENİ: arka-yan kontrolü
        self.min_sag_arka = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.teta = 0.0
        
        # --- PID PARAMETRELERİ ---
        self.Kp = 2.0
        self.Ki = 0.01
        self.Kd = 0.3
        self.toplam_hata = 0.0
        self.onceki_hata = 0.0
        
        self.hedef_teta = 0.0
        self.durum = "MOVE"
        self.backup_baslangic = 0.0
        self.donme_yonu = 1  # +1 sol, -1 sag
        
        rospy.spin()

    def guvenli_min(self, degerler):
        """inf ve nan değerlerini temizleyerek güvenli minimum döndürür."""
        temiz = [x for x in degerler if math.isfinite(x) and x > 0.01]
        return min(temiz) if temiz else 10.0  # Boşsa uzak say

    def lazerCallback(self, mesaj):
        ranges = mesaj.ranges

        # ÖN: -20° ile +20° arası (daha geniş ön görüş)
        on = list(ranges[0:20]) + list(ranges[340:360])
        
        # SOL: 20° - 90° arası
        sol = list(ranges[20:90])
        
        # SAĞ: 270° - 340° arası  
        sag = list(ranges[270:340])
        
        # ARKA-YAN: köşe kaçışı için
        sol_arka = list(ranges[90:135])
        sag_arka = list(ranges[225:270])

        self.min_on    = self.guvenli_min(on)
        self.min_sol   = self.guvenli_min(sol)
        self.min_sag   = self.guvenli_min(sag)
        self.min_sol_arka = self.guvenli_min(sol_arka)
        self.min_sag_arka = self.guvenli_min(sag_arka)

        self.HareketFonksiyonu()

    def odomCallback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.teta = yaw
        
    def kameraCallback(self, mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        cv2.imshow("Kamera", img)
        cv2.waitKey(1)
        
    def pid_hesapla(self, hedef, mevcut):
        hata = hedef - mevcut
        # Açıyı -pi ile +pi arasında tut
        hata = math.atan2(math.sin(hata), math.cos(hata))
        
        # Ki windup koruması
        if abs(hata) < 0.5:
            self.toplam_hata += hata
            self.toplam_hata = max(-2.0, min(2.0, self.toplam_hata))  # clamp
        
        hatanin_degisimi = hata - self.onceki_hata
        
        u = (self.Kp * hata) + (self.Ki * self.toplam_hata) + (self.Kd * hatanin_degisimi)
        
        # Maksimum dönüş hızını sınırla
        u = max(-1.5, min(1.5, u))
        
        self.onceki_hata = hata
        return u
        
    def HareketFonksiyonu(self):
        
        # =====================
        # 1. DURUM: DÜZ GİT
        # =====================
        if self.durum == "MOVE":
            
            # Her taraf kapalı → BACKUP
            if (self.min_on < 0.4 and self.min_sol < 0.4 and self.min_sag < 0.4):
                rospy.logwarn("KÖŞEYE SIKIŞTI → BACKUP")
                self.durum = "BACKUP"
                self.backup_baslangic = rospy.get_time()
                return

            # Ön engel → dön
            if self.min_on < 0.6:
                # Hangi taraf daha açık?
                if self.min_sol > self.min_sag:
                    self.donme_yonu = 1   # sola dön
                    aci = 1.4 + (0.6 - self.min_on)  # engele yakınlığa göre açı artar
                else:
                    self.donme_yonu = -1  # sağa dön
                    aci = -(1.4 + (0.6 - self.min_on))
                
                self.hedef_teta = self.teta + aci
                self.toplam_hata = 0.0
                self.onceki_hata = 0.0
                self.durum = "PID_ROTATE"
                rospy.loginfo(f"ENGEL → PID_ROTATE | Açı: {math.degrees(aci):.1f}°")
                return

            # Yan duvara çok yaklaşma (duvar takibi düzeltme)
            if self.min_sol < 0.35:
                self.hiz_mesaj.linear.x  = 0.1
                self.hiz_mesaj.angular.z = -0.4  # sağa it
            elif self.min_sag < 0.35:
                self.hiz_mesaj.linear.x  = 0.1
                self.hiz_mesaj.angular.z = 0.4   # sola it
            else:
                # Temiz yol → normal ilerleme
                self.hiz_mesaj.linear.x  = 0.2
                self.hiz_mesaj.angular.z = 0.0

        # =====================
        # 2. DURUM: PID İLE DÖN
        # =====================
        elif self.durum == "PID_ROTATE":
            
            # Dönüş sırasında da yan engel kontrolü
            if self.min_on < 0.3 and self.min_sol < 0.3 and self.min_sag < 0.3:
                rospy.logwarn("DÖNÜŞ SIRASINDA SIKIŞTI → BACKUP")
                self.durum = "BACKUP"
                self.backup_baslangic = rospy.get_time()
                return
            
            angular = self.pid_hesapla(self.hedef_teta, self.teta)
            
            # Dönüş sırasında çok yavaş lineer hareket (sıkışmayı azaltır)
            self.hiz_mesaj.linear.x  = 0.03
            self.hiz_mesaj.angular.z = angular
            
            hata_payi = math.atan2(
                math.sin(self.hedef_teta - self.teta),
                math.cos(self.hedef_teta - self.teta)
            )
            
            # Dönüş tamamlandı mı? (önde yeterli boşluk VE açı yakın)
            if abs(hata_payi) < 0.1 and self.min_on > 0.55:
                rospy.loginfo("DÖNÜŞ TAMAM → MOVE")
                self.durum = "MOVE"

        # =====================
        # 3. DURUM: GERİ VİTES
        # =====================
        elif self.durum == "BACKUP":
            gecen_sure = rospy.get_time() - self.backup_baslangic
            
            if gecen_sure < 1.2:  # 1.2 saniye geri git (daha uzun)
                self.hiz_mesaj.linear.x  = -0.15
                self.hiz_mesaj.angular.z = 0.0
            else:
                # Geri çıktıktan sonra en açık tarafa dön
                if self.min_sol_arka > self.min_sag_arka:
                    aci = 1.8   # sola geniş aç
                else:
                    aci = -1.8  # sağa geniş aç
                    
                self.hedef_teta = self.teta + aci
                self.toplam_hata = 0.0
                self.onceki_hata = 0.0
                self.durum = "PID_ROTATE"
                rospy.loginfo("BACKUP BİTTİ → PID_ROTATE")
                
        self.pub.publish(self.hiz_mesaj)

if __name__ == "__main__":
    try:
        TurtleBotKontrol()
    except rospy.ROSInterruptException:
        pass
