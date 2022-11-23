#[*] PythonからROS2を使用するためのモジュールを読み込みます．
import rclpy
from rclpy.node import Node

#[*] ステートマシーンを作成するためのモジュールです．
import smach

#[*] サービス通信を行うための型を読み込みます．
from airobot_interfaces.srv import StringCommand


# Bring meタスクのステートマシーンを実行するノードを定義
class Bringme_state(Node):
    def __init__(self):
        #[*] ノード名を bringme_state として登録します．
        super().__init__('bringme_state')

    def execute(self):
        # Smachステートマシーンを作成
        sm = smach.StateMachine(outcomes=['succeeded'])

        #[*] 状態同士のつながりを定義します．
        with sm:
            # コンテナに状態を追加
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
    #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    rclpy.init()
    
    #[*] ステートマシーンのノードを初期化します．
    node = Bringme_state()
    
    #[*] ステートマシーンを実行します．
    node.execute()


# 音声認識関連の状態
class Voice(smach.State):
    def __init__(self, node):
        #[*] 音声認識関連の状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
        smach.State.__init__(
            self,
            output_keys=['target_object', 'target_location'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        
        #[*] ロガーを定義します．
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'voice/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        #[*] 音声認識関連の状態が実行されたら，ログを残します．
        self.logger.info('音声認識の状態を開始します')

        #[*] 音声認識関連のサービスを実行するようにリクエストを送信します．
        self.req.command = 'start'
        result = self.send_request()

        #[*] 音声認識で認識結果が得られた場合
        if result:
            #[*] 認識した物体と場所の名前を，他の状態に値を渡す際の変数に入力します．
            userdata.target_object = self.target_object
            userdata.target_location = self.target_location

            #[*] 成功を返します．
            return 'succeeded'
        else:
            #[*] 失敗を返します．
            return 'failed'

    def send_request(self):
        #[*] サーバと通信を行うためのクライアントを作成します．
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        #[*] レスポンスのanswer変数に文字が入っている場合
        if response.answer != '':
            #[*] クラス内変数に文字を代入します．
            self.target_object = 'cup'  # find_object_name(response.answer)
            self.target_location = 'kitchen'  # find_location_name(response.answer)

            return True
        else:
            return False


# ナビゲーションの状態
class Navigation(smach.State):
    def __init__(self, node):
        #[*] ナビゲーションの状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
        smach.State.__init__(
            self,
            input_keys=['target_location'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        
        #[*] ロガーを定義します．
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'navigation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        #[*] ナビゲーションの状態が実行されたら，ログを残します．
        self.logger.info('ナビゲーションの状態を開始します')

        #[*] ナビゲーションのサービスを実行するようにリクエストを送信します．
        self.req.command = userdata.target_location
        result = self.send_request()

        #[*] 移動先に到達した場合
        if result:
            #[*] 成功を返します．
            return 'succeeded'
        else:
            #[*] 失敗を返します．
            return 'failed'

    def send_request(self):
        #[*] サーバと通信を行うためのクライアントを作成します．
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        #[*] レスポンスのanswer変数に reached が入っている場合
        if response.answer == 'reached':
            return True
        else:
            return False


# ビジョンの状態
class Vision(smach.State):
    def __init__(self, node):
        #[*] ビジョンの状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
        smach.State.__init__(
            self,
            input_keys=['target_object'],
            output_keys=['target_object_pos'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        
        #[*] ロガーを定義します．
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'vision/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        #[*] ビジョンの状態が実行されたら，ログを残します．
        self.logger.info('ビジョンの状態を開始します')

        #[*] 物体認識のサービスを実行するようにリクエストを送信します．
        self.req.command = userdata.target_object
        result = self.send_request()
        
        #[*] 物体認識が成功したとして，物体の位置を代入しています．
        userdata.target_object_pos = [0.12, -0.03, 0.4]  # 単位は[m]

        #[*] 物体を認識した場合
        if result:
            return 'succeeded'
        else:
            return 'failed'

    def send_request(self):
        #[*] サーバと通信を行うためのクライアントを作成します．
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        #[*] レスポンスのdetected変数に detected が入っている場合
        if response.answer == 'detected':
            #[*] 成功を返します．
            return True
        else:
            #[*] 失敗を返します．
            return False


# マニピュレーションの状態
class Manipulation(smach.State):
    def __init__(self, node):
        #[*] マニピュレーションの状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
        smach.State.__init__(
            self,
            input_keys=['target_object_pos'],
            outcomes=['exit', 'failed'])

        # Nodeを作成
        self.node = node
        
        #[*] ロガーを定義します．
        self.logger = self.node.get_logger()

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(
            StringCommand, 'manipulation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        #[*] マニピュレーションの状態が実行されたら，ログを残します．
        self.logger.info('マニピュレーションの状態を開始します')

        #[*] アームを伸ばす先の座標を代入します．
        target_object_pos = userdata.target_object_pos
        print(f'{target_object_pos}')

        #[*] マニピュレーションのサービスを実行するようにリクエストを送信します．
        self.req.command = 'start'
        result = self.send_request()

        #[*] アームのリーチングが完了した場合
        if result:
            return 'exit'
        else:
            return 'failed'

    def send_request(self):
        #[*] サーバと通信を行うためのクライアントを作成します．
        self.future = self.cli.call_async(self.req)

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        #[*] レスポンスの変数に reached が入っている場合
        if response.answer == 'reached':
            #[*] 成功を返します．
            return True
        else:
            #[*] 失敗を返します．
            return False
