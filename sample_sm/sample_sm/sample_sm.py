# ファイル名：sample_sm.py

import rclpy #[*] PythonからROS2を使用するためのモジュールを読み込みます．
from rclpy.node import Node
import smach #[*] ステートマシーンを作成するためのモジュールです．


# 探索の状態を定義します．
class Search(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['succeeded', 'finished']) #[*] 探索の状態における結果を事前に定義します．
        self.counter = 0 #[*] 何回この状態に到達したのかをカウントする変数です．
        self.logger = _node.get_logger() #[*] ロガーを定義します．

    def execute(self, userdata):
        self.logger.info('探索中です') #[*] 探索の状態にいることをログに残します．
        if self.counter < 3:
            self.logger.info('スイーツを見つけました！') #[*] 探索の状態へ訪れた回数が3回未満の場合
            self.counter += 1
            return 'succeeded' #[*] 'succeeded'という結果を返します．
        else:
            self.logger.info('お腹いっぱいです・・・') #[*] 探索の状態へ訪れた回数が3回になった場合
            return 'finished' #[*] 'finished'という結果を返します．

        
# 食事の状態を定義します．
class Eat(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['done']) #[*] 食事の状態における結果を事前に定義します．
        self.logger = _node.get_logger() #[*] ロガーを定義します．

    def execute(self, userdata):
        self.logger.info('食べてます！') #[*] 食事の状態にいることをログに残します．
        return 'done' #[*] 'done'という結果を返します．


# ステートマシーンを実行するノードを定義します．
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine') #[*] ノード名をstate_machineとして登録します．

    def execute(self):
        # Smachステートマシーンを作成
        sm = smach.StateMachine(outcomes=['end'])
        # コンテナを開く
        with sm: #[*] 状態同士のつながりを定義します．
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
    rclpy.init() #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    node = StateMachine() #[*] ステートマシーンのノードを初期化します．
    node.execute() #[*] ステートマシーンを実行します．
