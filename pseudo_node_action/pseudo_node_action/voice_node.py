import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from airobot_interfaces.action import StringCommand

import random
from time import sleep


class VoiceActionServer(Node):
    def __init__(self):
        super().__init__('voice_action_server')
        self._voice_action_server = ActionServer(
            self,
            StringCommand,
            'ps_voice/command',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'`{goal_handle.request.command}`秒数内に音声認識を開始します')

        result_msg = StringCommand.Result()
        result_msg.answer = ''

        feedback_msg = StringCommand.Feedback()
        feedback_msg.process = ''

        wait_time = int(goal_handle.request.command)

        # --- この部分は未知です ---
        unk_text = 'bring me a cup from the kitchen'
        unk_text_split = unk_text.split()
        unk_text_list = unk_text_split + [''] * (wait_time - len(unk_text_split))
        temp_out_list = []
        temp_out_text = ''
        # --------------------------

        for i in range(wait_time):
            sleep(1)
            prob = random.random()

            if 0.95 > prob:
                self.get_logger().info('音声認識をしています')
                temp_out_list.append(unk_text_list[i]+ ' ' if unk_text_list[i]!='' else '')
                temp_out_text = (''.join(word for word in temp_out_list if word))
                
                feedback_msg.process = temp_out_text
                goal_handle.publish_feedback(feedback_msg)

            else:
                self.get_logger().info('音声認識が失敗しました')

                feedback_msg.process = ''                
                goal_handle.publish_feedback(feedback_msg)
                goal_handle.abort()
                
                result_msg.answer = 'failed'
                return result_msg
        
        self.get_logger().info('音声認識が成功しました')

        goal_handle.succeed()
        
        result_msg.answer = feedback_msg.process
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    voice_action_server = VoiceActionServer()

    rclpy.spin(voice_action_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
