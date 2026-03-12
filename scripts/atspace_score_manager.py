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


class STARTTASKState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Start Task State ===')
        try:
            if rospy.get_time() > 300:
                return 'timeout'
            return 'success'
        except Exception as e:
            rospy.logerr('Start task error: %s', str(e))
            return 'fail'

class NavigationTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Navigation Task State ===')
        try:
            if rospy.get_time() > 300:
                return 'timeout'
            return 'success'
        except Exception as e:
            rospy.logerr('Navigation task error: %s', str(e))
            return 'fail'

class SearchTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Search Task State ===')
        try:
            if rospy.get_time() > 300:
                return 'timeout'
            return 'success'
        except Exception as e:
            rospy.logerr('Search task error: %s', str(e))
            return 'fail'

class DockingTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Docking Task State ===')
        try:
            if rospy.get_time() > 300:
                return 'timeout'
            return 'success'
        except Exception as e:
            rospy.logerr('Docking task error: %s', str(e))
            return 'fail'

class FinishState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Finish State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('Finish state error: %s', str(e))
            return 'fail'

def main():    
    goal_checker = GoalChecker()
    object_detect_checker = ObjectDetectChecker()
    
    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])
    
    with sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'success': 'STARTTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'})
        
        smach.StateMachine.add('STARTTASK', StartTaskState(),
                               transitions={'success': 'NAVIGATIONTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'})

        smach.StateMachine.add('NAVIGATIONTASK', NavigationTaskState(),
                               transitions={'success': 'SEARCHTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'})

        smach.StateMachine.add('SEARCHTASK', SearchTaskState(),
                               transitions={'success': 'DOCKINGTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'})

        smach.StateMachine.add('DOCKINGTASK', DockingTaskState(),
                               transitions={'success': 'FINISH',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'})
        
        smach.StateMachine.add('FINISH', FinishState(),
                               transitions={'success': 'SUCCESS',
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
