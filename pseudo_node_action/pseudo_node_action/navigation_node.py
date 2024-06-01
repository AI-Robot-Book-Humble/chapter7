import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from airobot_interfaces.action import StringCommand

import random
from time import sleep


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._nav_action_server = ActionServer( # ActionServerを作成します
            self,                               # ROSノードを指定します
            StringCommand,                      # Action型を指定します
            'ps_navigation/command',            # Action名を指定します
            self.execute_callback)              # 送信されたgoalをcallback関数に処理します

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'目標座標 `{goal_handle.request.command}` への移動を開始します')

        result_msg = StringCommand.Result()     # 入力されたgoalに対する結果の型を定義します
        result_msg.answer = ''                  # Actionの結果を初期化します

        feedback_msg = StringCommand.Feedback() # 入力されたgoalに対する途中結果の型を定義します
        feedback_msg.process = ''               # Actionの途中結果を初期化します

        wait_time = 10 # 仮処理に必要な秒数を定義します

        for i in range(wait_time):
            # ナビゲーションの処理を行います

            sleep(1) # 処理を可視化するために，1秒停止します
            prob = random.random() # [0,1]の値を抽出します

            if 0.95 > prob:
                # [成功] probの値は0.95より小さい場合，実行されます（95%）
                self.get_logger().info(f'目標座標 `{goal_handle.request.command}` へ移動しています')

                feedback_msg.process = 'navigating' if i < wait_time-1 else 'reached' # 途中結果として，`navigating`とし，最終結果として`reached`とします
                goal_handle.publish_feedback(feedback_msg) # 途中結果をフィードバックとしてpublishします

            else:
                # [失敗] probの値は0.95以上の場合，実行されます（5%）
                self.get_logger().info(f'目標座標 `{goal_handle.request.command}` への移動が失敗しました')

                feedback_msg.process = '' # 途中に何らかの失敗が生じたため，フィードバックを空とします
                goal_handle.publish_feedback(feedback_msg) # 途中結果をフィードバックとしてpublishします
                goal_handle.abort() # 処理の途中に失敗したため，Action処理を強制終了します

                result_msg.answer = 'failed' # 失敗したため，最終結果を`failed`とします
                return result_msg

        self.get_logger().info(f'目標座標 `{goal_handle.request.command}` への移動が成功しました')

        goal_handle.succeed() # goalが成功したことを報告します

        result_msg.answer = feedback_msg.process # 途中結果の最後の値を`result_msg`に入力します
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    navigation_action_server = NavigationActionServer() # NavigationActionServer()というClassを宣伝します

    rclpy.spin(navigation_action_server) # ActionServerのcallback関数を起動します

    rclpy.shutdown()

if __name__ == '__main__':
    main()
