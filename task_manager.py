#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager_node')
        
        # QR Servis Bağlantısı
        rospy.wait_for_service('/decode_qr')
        self.qr_client = rospy.ServiceProxy('/decode_qr', Trigger)
        
        # Navigasyon (MoveBase) Bağlantısı
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        
        # Lokasyon listesini parametre sunucusundan al (global)
        self.locations = rospy.get_param('locations', [])
        rospy.loginfo(f"Yüklenen Lokasyonlar: {self.locations}")
        
    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()

    def run(self):
        for loc_name in self.locations:
            try:
                rospy.loginfo(f">>> Sıradaki hedef: {loc_name} <<<")
                # Parametre sunucusundaki koordinatları çek
                data = rospy.get_param(loc_name)
                g = data['goal']
                
                # Hedefe git
                self.send_goal(g['x'], g['y'], g['yaw'])
                
                # QR Kodunu oku
                res = self.qr_client()
                if res.success:
                    rospy.loginfo(f"QR Başarılı: {res.message}")
                else:
                    rospy.logwarn(f"QR Uyarısı: {res.message}")
            except Exception as e:
                rospy.logerr(f"Hata oluştu: {str(e)}")

if __name__ == '__main__':
    try:
        manager = TaskManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
