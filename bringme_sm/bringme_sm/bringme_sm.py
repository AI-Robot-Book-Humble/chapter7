import rclpy
from rclpy.node import Node
import smach

from airobot_interfaces.srv import StringCommand


# Bring meタスクのステートマシーンを実行するノードを定義
class Bringme_state(Node):
    def __init__(self):
        super().__init__('bringme_state')

    def execute(self):
        # Smachステートマシーンを作成
        sm = smach.StateMachine(outcomes=['succeeded'])

        # コンテナに状態を追加
        with sm:
            smach.StateMachine.add(
                'VOICE',
                Voice(self),
                {'succeeded': 'NAVIGATION', 'failed': 'VOICE'})

            smach.StateMachine.add(
                'NAVIGATION',
                Navigation(self),
                {'succeeded': 'VISION', 'failed': 'NAVIGATION'})

            smach.StateMachine.add(
                'VISION',
                Vision(self),
                {'succeeded': 'MANIPULATION', 'failed': 'VISION'})

            smach.StateMachine.add(
                'MANIPULATION',
                Manipulation(self),
                {'failed': 'VISION', 'exit': 'succeeded'})

        # Smachプランを実行
        sm.execute()


def main():
    rclpy.init()
    node = Bringme_state()
    node.execute()


# 音声認識関連の状態
class Voice(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            output_keys=['target_object', 'target_location'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'voice/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('音声認識の状態を開始します')

        self.req.command = 'start'
        result = self.send_request()

        if result:
            userdata.target_object = self.target_object
            userdata.target_location = self.target_location

            return 'succeeded'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer != '':
            self.target_object = 'cup'  # find_object_name(response.answer)
            self.target_location = 'kitchen'  # find_location_name(response.answer)

            return True
        else:
            return False


# ナビゲーションの状態
class Navigation(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            input_keys=['target_location'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'navigation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('ナビゲーションの状態を開始します')

        self.req.command = userdata.target_location
        result = self.send_request()

        if result:
            return 'succeeded'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'reached':
            return True
        else:
            return False


# ビジョンの状態
class Vision(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            input_keys=['target_object'],
            output_keys=['target_object_pos'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'vision/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('ビジョンの状態を開始します')

        self.req.command = userdata.target_object
        result = self.send_request()
        userdata.target_object_pos = [0.12, -0.03, 0.4]  # 単位は[m]

        if result:
            return 'succeeded'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'detected':
            return True
        else:
            return False


# マニピュレーションの状態
class Manipulation(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            input_keys=['target_object_pos'],
            outcomes=['exit', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(
            StringCommand, 'manipulation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('マニピュレーションの状態を開始します')

        target_object_pos = userdata.target_object_pos
        print(f'{target_object_pos}')

        self.req.command = 'start'
        result = self.send_request()

        if result:
            return 'exit'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'reached':
            return True
        else:
            return False
