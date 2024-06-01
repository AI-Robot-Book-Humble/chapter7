import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from airobot_interfaces.action import StringCommand

import random
from time import sleep


class VoiceActionServer(Node):
    def __init__(self):
        super().__init__('voice_action_server')
        self._voice_action_server = ActionServer(   # ActionServerを作成します
            self,                                   # ROSノードを指定します
            StringCommand,                          # Action型を指定します
            'ps_voice/command',                     # Action名を指定します
            self.execute_callback)                  # 送信されたgoalをcallback関数に処理します

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'`{goal_handle.request.command}`秒数内に音声認識を開始します')

        result_msg = StringCommand.Result()     # 入力されたgoalに対する結果の型を定義します
        result_msg.answer = ''                  # Actionの結果を初期化します

        feedback_msg = StringCommand.Feedback() # 入力されたgoalに対する途中結果の型を定義します
        feedback_msg.process = ''               # Actionの途中結果を初期化します

        wait_time = int(goal_handle.request.command) # 送信されたコマンドはString型のため，int型に変換します

        # --- この部分は音声処理の代わりに，事前に定義します．実際では未知です ---
        unk_text = 'bring me a cup from the kitchen'    # 音声認識による音声データからテキストに変換します
        unk_text_split = unk_text.split()               # 言葉ごとをList型に入力します
        unk_text_list = unk_text_split + [''] * (wait_time - len(unk_text_split)) # Listの最後にwait_timeの長さに応じて，空の要素を入力します
        temp_out_list = [] # 途中結果のため，仮Listを定義します
        temp_out_text = '' # 途中結果のため，仮テキストの変数を定義します
        # ------------------------------------------------------------------------

        for i in range(wait_time):
            # 音声認識の処理を行います

            sleep(1) # 処理を可視化するために，1秒停止します
            prob = random.random() # [0,1]の値を抽出します

            if 0.95 > prob:
                # [成功] probの値は0.95より小さい場合，実行されます（95%）
                self.get_logger().info('音声認識をしています')
                temp_out_list.append(unk_text_list[i]+ ' ' if unk_text_list[i]!='' else '') # 仮Listにunk_text_listの要素が空でなければ，追加します
                temp_out_text = (''.join(word for word in temp_out_list if word)) # List型からString型に変換します
                
                feedback_msg.process = temp_out_text # 途中結果として，`temp_out_text`を入力します
                goal_handle.publish_feedback(feedback_msg) # 途中結果をフィードバックとしてpublishします

            else:
                # [失敗] probの値は0.95以上の場合，実行されます（5%）
                self.get_logger().info('音声認識が失敗しました')

                feedback_msg.process = '' # 途中に何らかの失敗が生じたため，フィードバックを空とします
                goal_handle.publish_feedback(feedback_msg) # 途中結果をフィードバックとしてpublishします
                goal_handle.abort() # 処理の途中に失敗したため，Action処理を強制終了します

                result_msg.answer = 'failed' # 失敗したため，最終結果を`failed`とします
                return result_msg
        
        self.get_logger().info('音声認識が成功しました')

        goal_handle.succeed() # goalが成功したことを報告します

        result_msg.answer = feedback_msg.process # 途中結果の最後の値を`result_msg`に入力します
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    voice_action_server = VoiceActionServer() # VoiceActionServer()というClassを宣伝します

    rclpy.spin(voice_action_server) # ActionServerのcallback関数を起動します

    rclpy.shutdown()

if __name__ == '__main__':
    main()
