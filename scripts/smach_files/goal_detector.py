#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import geometry_msgs.msg

class GoalDetectorTF:
    def __init__(self):
        rospy.init_node('goal_detector_node')

        self.x_range = [10.0, 12.0]
        self.y_range = [-10.0, -8.0] 
        self.z_range = [4.0, 6.0]

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.score     = 0
        self.is_inside = False

        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_position)

    def check_position(self, event):
        try:
            trans = self.tf_buffer.lookup_transform('iss_body', 'body', rospy.Time(0))

            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z
            

            in_x = abs(self.x_range[0]) < abs(curr_x) < abs(self.x_range[1])
            in_y = abs(self.y_range[0]) < abs(curr_y) < abs(self.y_range[1])
            in_z = abs(self.z_range[0]) < abs(curr_z) < abs(self.z_range[1])
            # rospy.loginfo("Current Position: (%.2f, %.2f, %.2f) | In Box: X:%s Y:%s Z:%s", 
            #               curr_x, curr_y, curr_z, in_x, in_y, in_z)

            if in_x and in_y and in_z:
                if not self.is_inside:
                    self.score += 10
                    self.is_inside = True
                    rospy.loginfo("目標位置に到達しました! Score: %d", self.score)
            # else:
            #     if self.is_inside:
            #         rospy.loginfo("目標位置から離れました")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

if __name__ == '__main__':
    detector = GoalDetectorTF()
    rospy.spin()