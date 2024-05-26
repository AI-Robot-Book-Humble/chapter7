import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from airobot_interfaces.action import StringCommand

import random
from time import sleep


class ManipulationActionServer(Node):

    def __init__(self):
        super().__init__('manipulation_action_server')
        self.mani_action_server = ActionServer(
            self,
            StringCommand,
            'ps_manipulation/command',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'目標物体 `{goal_handle.request.command}` の把持を開始します')
        
        result_msg = StringCommand.Result()
        result_msg.answer = ''

        feedback_msg = StringCommand.Feedback()
        feedback_msg.process = ''

        wait_time = 10

        for i in range(wait_time):
            sleep(1)
            prob = random.random()

            if 0.95 > prob:
                self.get_logger().info(f'目標物体 `{goal_handle.request.command}` を把持しようとしています')

                feedback_msg.process = 'reaching' if i < wait_time-1 else 'reached'
                goal_handle.publish_feedback(feedback_msg)

            else:
                self.get_logger().info(f'目標物体 `{goal_handle.request.command}` の把持が失敗しました')
                
                feedback_msg.process = ''
                goal_handle.publish_feedback(feedback_msg)
                goal_handle.abort() # check

                result_msg.answer = 'failed'
                return result_msg

        self.get_logger().info(f'目標物体 `{goal_handle.request.command}` の把持を成功しました')

        goal_handle.succeed()

        result_msg.answer = feedback_msg.process
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    manipulation_action_server = ManipulationActionServer()

    rclpy.spin(manipulation_action_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
