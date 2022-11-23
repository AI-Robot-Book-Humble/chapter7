# ファイル名：sample_sm.py
#[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy
from rclpy.node import Node

#[*] ステートマシーンを作成するためのモジュールです．
import smach


# 探索の状態を定義します．
class Search(smach.State):
    def __init__(self, _node):
        #[*] 探索の状態における結果を事前に定義します．
        smach.State.__init__(self, outcomes=['succeeded', 'finished'])
        
        #[*] 何回この状態に到達したのかをカウントする変数です．
        self.counter = 0
        
        #[*] ロガーを定義します．
        self.logger = _node.get_logger()

    def execute(self, userdata):
        #[*] 探索の状態にいることをログに残します．
        self.logger.info('探索中です')
        
        if self.counter < 3:
            #[*] 探索の状態へ訪れた回数が3回未満の場合
            self.logger.info('スイーツを見つけました！')
            self.counter += 1
            
            #[*] 'succeeded'という結果を返します．
            return 'succeeded'
        else:
            #[*] 探索の状態へ訪れた回数が3回になった場合
            self.logger.info('お腹いっぱいです・・・')
            
            #[*] 'finished'という結果を返します．
            return 'finished'


# 食事の状態を定義します．
class Eat(smach.State):
    def __init__(self, _node):
        #[*] 食事の状態における結果を事前に定義します．
        smach.State.__init__(self, outcomes=['done'])
        
        #[*] ロガーを定義します．
        self.logger = _node.get_logger()

    def execute(self, userdata):
        #[*] 食事の状態にいることをログに残します．
        self.logger.info('食べてます！')
        
        #[*] 'done'という結果を返します．
        return 'done'


# ステートマシーンを実行するノードを定義します．
class StateMachine(Node):
    def __init__(self):
        #[*] ノード名をstate_machineとして登録します．
        super().__init__('state_machine')

    def execute(self):
        # Smachステートマシーンを作成
        sm = smach.StateMachine(outcomes=['end'])
        
        #[*] 状態同士のつながりを定義します．
        with sm:
            # コンテナに状態を追加
            smach.StateMachine.add(
                'SEARCH', Search(self),
                transitions={'succeeded': 'EAT', 'finished': 'end'})
            smach.StateMachine.add(
                'EAT', Eat(self),
                transitions={'done': 'SEARCH'})

        # Smachプランを実行
        outcome = sm.execute()
        self.get_logger().info(f'outcom: {outcome}')


def main():
    #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    rclpy.init()
    
    #[*] ステートマシーンのノードを初期化します．
    node = StateMachine()
    
    #[*] ステートマシーンを実行します．
    node.execute()
