#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
from smach_files.goal_checker import GoalChecker

class InitialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], output_keys=['initial_score'])
        self.score     = rospy.get_param('~competition/initial_score', 0)
        self.competition_time_limit = rospy.get_param('~competition/time_limit', 300)
    
    def execute(self, userdata):
        rospy.loginfo('=== Initial State ===')
        try:
            userdata.initial_score = self.score
            return 'success'
        except Exception as e:
            rospy.logerr('System initialization error: %s', str(e))
            return 'fail'


class StartTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'], input_keys=['start_task_score'], output_keys=['start_task_score'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Start Task State ===')
        try:
            userdata.start_task_score += 10
            return 'success'
        except Exception as e:
            rospy.logerr('Start task error: %s', str(e))
            return 'fail'

class NavigationTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'], input_keys=['navigation_task_score'], output_keys=['navigation_task_score'])
    
    def execute(self, userdata):
        rospy.loginfo('=== Navigation Task State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('Navigation task error: %s', str(e))
            return 'fail'

class SearchTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'], input_keys=['search_task_score'], output_keys=['search_task_score'])

    def execute(self, userdata):
        rospy.loginfo('=== Search Task State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('Search task error: %s', str(e))
            return 'fail'

class DockingTaskState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'timeout', 'fail'], input_keys=['docking_task_score'], output_keys=['docking_task_score'])

    def execute(self, userdata):
        rospy.loginfo('=== Docking Task State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('Docking task error: %s', str(e))
            return 'fail'

class FinishState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['finish_score'])
    def execute(self, userdata):
        rospy.loginfo('=== Finish State ===')
        try:
            return 'success'
        except Exception as e:
            rospy.logerr('Finish state error: %s', str(e))
            return 'fail'

def main():    
    
    sm = smach.StateMachine(outcomes=['SUCCESS', 'FAIL'])
    
    with sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'success': 'STARTTASK',
                                           'fail': 'FAIL'},
                               remapping={'initial_score': 'score'})
        
        smach.StateMachine.add('STARTTASK', StartTaskState(),
                               transitions={'success': 'NAVIGATIONTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'},
                               remapping={'start_task_score': 'score'})

        smach.StateMachine.add('NAVIGATIONTASK', NavigationTaskState(),
                               transitions={'success': 'SEARCHTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'},
                               remapping={'navigation_task_score': 'score'})

        smach.StateMachine.add('SEARCHTASK', SearchTaskState(),
                               transitions={'success': 'DOCKINGTASK',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'},
                               remapping={'search_task_score': 'score'})

        smach.StateMachine.add('DOCKINGTASK', DockingTaskState(),
                               transitions={'success': 'FINISH',
                                            'timeout': 'FINISH',
                                           'fail': 'FAIL'},
                               remapping={'docking_task_score': 'score'})
        
        smach.StateMachine.add('FINISH', FinishState(),
                               transitions={'success': 'SUCCESS',
                                           'fail': 'FAIL'},
                               remapping={'finish_score': 'score'})
    
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
