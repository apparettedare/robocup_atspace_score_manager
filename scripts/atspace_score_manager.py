#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
from smach_files.goal_detector import GoalDetectorTF

class InitialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'error'])
    
    def execute(self, userdata):
        rospy.loginfo('初期状態: システム起動中...')
        try:
            rospy.sleep(1.0)
            rospy.loginfo('システムが準備完了しました')
            return 'ready'
        except Exception as e:
            rospy.logerr('システム初期化エラー: %s', str(e))
            return 'error'


class MoveToGoalState(smach.State):
    def __init__(self, goal_detector):
        smach.State.__init__(self, outcomes=['goal_reached', 'timeout', 'error'])
        self.goal_detector = goal_detector
        self.start_time = None
        self.timeout_duration = 600.0
    
    def execute(self, userdata):
        self.start_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                # 目標位置に到達したか確認
                if self.goal_detector.is_inside:
                    rospy.loginfo('目標位置に到達しました!')
                    return 'goal_reached'
                
                # タイムアウト確認
                elapsed = (rospy.Time.now() - self.start_time).to_sec()
                if elapsed > self.timeout_duration:
                    rospy.logwarn('目標位置への移動がタイムアウトしました')
                    return 'timeout'
                
                rospy.sleep(0.5)
        
        except Exception as e:
            rospy.logerr('移動状態エラー: %s', str(e))
            return 'error'


class CheckPositionState(smach.State):
    """
    目標位置確認状態: 目標位置内にいることを確認
    """
    def __init__(self, goal_detector):
        smach.State.__init__(self, outcomes=['confirmed', 'left_area', 'error'])
        self.goal_detector = goal_detector
        self.check_duration = 5.0  # 5秒間確認
    
    def execute(self, userdata):
        rospy.loginfo('確認状態: 目標位置に到達したことを確認中...')
        start_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                if not self.goal_detector.is_inside:
                    rospy.logwarn('目標位置から離れました')
                    return 'left_area'
                
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > self.check_duration:
                    rospy.loginfo('目標位置での滞在を確認しました!')
                    return 'confirmed'
                
                rospy.sleep(0.5)
        
        except Exception as e:
            rospy.logerr('確認状態エラー: %s', str(e))
            return 'error'


class ScoringState(smach.State):
    """
    スコアリング状態: 目標到達時のスコアを獲得
    """
    def __init__(self, goal_detector):
        smach.State.__init__(self, outcomes=['success', 'error'])
        self.goal_detector = goal_detector
    
    def execute(self, userdata):
        rospy.loginfo('スコアリング状態: スコアを計算中...')
        
        try:
            final_score = self.goal_detector.score
            rospy.loginfo('最終スコア: %d', final_score)
            return 'success'
        
        except Exception as e:
            rospy.logerr('スコアリングエラー: %s', str(e))
            return 'error'


class ErrorState(smach.State):
    """
    エラー状態: エラー処理
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset', 'shutdown'])
    
    def execute(self, userdata):
        rospy.logerr('エラーが発生しました。システムをリセットします')
        rospy.sleep(2.0)
        return 'reset'


def main():
    """
    メイン関数: SMACH状態機械の初期化と実行
    """
    rospy.init_node('robocup_atspace_score_manager_node')
    
    goal_detector = GoalDetectorTF()
    
    sm = smach.StateMachine(outcomes=['success'])
    
    with sm:
        smach.StateMachine.add('INITIAL', InitialState(),
                               transitions={'ready': 'MOVE_TO_GOAL',
                                           'error': 'ERROR'})
        
        smach.StateMachine.add('MOVE_TO_GOAL', MoveToGoalState(goal_detector),
                               transitions={'goal_reached': 'CHECK_POSITION',
                                           'timeout': 'ERROR',
                                           'error': 'ERROR'})
        
        smach.StateMachine.add('CHECK_POSITION', CheckPositionState(goal_detector),
                               transitions={'confirmed': 'SCORING',
                                           'left_area': 'MOVE_TO_GOAL',
                                           'error': 'ERROR'})
        
        smach.StateMachine.add('SCORING', ScoringState(goal_detector),
                               transitions={'success': 'success',
                                           'error': 'ERROR'})
        
        smach.StateMachine.add('ERROR', ErrorState(),
                               transitions={'reset': 'INITIAL',
                                           'shutdown': 'failure'})
    
    # ビジュアライゼーション用に状態機械を設定
    sis = smach_ros.IntrospectionServer('robocup_atspace_score_manager_smach', sm, '/robocup_atspace_score_manager')
    sis.start()
    
    # ステートマシンの実行
    rospy.loginfo('ATSPACE スコアマネジャーを開始します')
    outcome = sm.execute()
    
    rospy.loginfo('スコアマネジャー終了: %s', outcome)
    sis.stop()
    
    return outcome


if __name__ == '__main__':
    try:
        outcome = main()
        rospy.loginfo('プログラム終了: %s', outcome)
    except rospy.ROSInterruptException:
        rospy.loginfo('ROSが中断されました')
    except Exception as e:
        rospy.logerr('予期しないエラー: %s', str(e))
