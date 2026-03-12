#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
from smach_files.goal_checker import GoalChecker

class InitialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Initial State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('System initialization error: %s', str(e))
            return 'fail'


class CheckGoalState(smach.State):
    def __init__(self, goal_checker):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'])
        self.goal_checker = goal_checker
        self.start_time = None
        self.timeout_duration = 600.0
    
    def execute(self, userdata):
        self.start_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                # 目標位置に到達したか確認
                if self.goal_checker.is_inside:
                    rospy.loginfo('目標位置に到達しました!')
                    return 'success'
                
                # タイムアウト確認
                elapsed = (rospy.Time.now() - self.start_time).to_sec()
                if elapsed > self.timeout_duration:
                    rospy.logwarn('目標位置への移動がタイムアウトしました')
                    return 'timeout'
                
                rospy.sleep(0.5)
        
        except Exception as e:
            rospy.logerr('移動状態エラー: %s', str(e))
            return 'fail'


class CheckObjectDetectState(smach.State):
    def __init__(self, goal_checker):
        smach.State.__init__(self, outcomes=['confirmed', 'left_area', 'fail'])
        self.goal_checker = goal_checker
        self.check_duration = 5.0  # 5秒間確認
    
    def execute(self, userdata):
        rospy.loginfo('確認状態: 目標位置に到達したことを確認中...')
        start_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                if not self.goal_checker.is_inside:
                    rospy.logwarn('目標位置から離れました')
                    return 'left_area'
                
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > self.check_duration:
                    rospy.loginfo('目標位置での滞在を確認しました!')
                    return 'confirmed'
                
                rospy.sleep(0.5)
        
        except Exception as e:
            rospy.logerr('確認状態エラー: %s', str(e))
            return 'fail'


class FinishState(smach.State):
    def __init__(self, goal_checker):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.goal_checker = goal_checker
    
    def execute(self, userdata):
        rospy.loginfo('スコアリング状態: スコアを計算中...')
        
        try:
            final_score = self.goal_checker.score
            rospy.loginfo('最終スコア: %d', final_score)
            return 'success'
        
        except Exception as e:
            rospy.logerr('スコアリングエラー: %s', str(e))
            return 'fail'


def main():    
    goal_checker = GoalChecker()
    object_detect_checker = ObjectDetectChecker()
    
    sm = smach.StateMachine(outcomes=['success', 'fail'])
    
    with sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'success': 'CHECK_GOAL',
                                           'fail': 'FAIL'})
        
        smach.StateMachine.add('CHECK_GOAL', CheckGoalState(goal_checker),
                               transitions={'check_object_detect': 'CHECK_OBJECT_DETECT',
                                            'finish': 'FINISH',
                                           'timeout': 'FINISH',
                                           'fail': 'FAIL'})
        
        smach.StateMachine.add('CHECK_OBJECT_DETECT', CheckObjectDetectState(object_detect_checker),
                               transitions={'success': 'CHECK_GOAL',
                                           'timeout': 'FINISH',
                                           'fail': 'FAIL'})
        
        smach.StateMachine.add('FINISH', FinishState(),
                               transitions={'success': 'success',
                                           'fail': 'FAIL'})
    
    sis = smach_ros.IntrospectionServer('robocup_atspace_score_manager_smach', sm, '/robocup_atspace_score_manager')
    sis.start()
    
    rospy.loginfo('RoboCup@Space Score Manager is running...')
    outcome = sm.execute()
    
    rospy.loginfo('Score Manager finished: %s', outcome)
    sis.stop()
    


if __name__ == '__main__':
    try:
        rospy.init_node('robocup_atspace_score_manager_node')
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program interrupted by user')
    except Exception as e:
        rospy.logerr('Unexpected error: %s', str(e))
