import rclpy #[*] PythonからROS2を使用するためのモジュールを読み込みます．
from rclpy.node import Node
import smach #[*] ステートマシーンを作成するためのモジュールです．

from airobot_interfaces.srv import StringCommand #[*] サービス通信を行うための型を読み込みます．


# Bring meタスクのステートマシーンを実行するノードを定義
class Bringme_state(Node):
    def __init__(self):
        super().__init__('bringme_state') #[*] ノード名を bringme_state として登録します．

    def execute(self):
        # Smachステートマシーンを作成
        sm = smach.StateMachine(outcomes=['succeeded'])

        # コンテナに状態を追加
        with sm: #[*] 状態同士のつながりを定義します．
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
    rclpy.init() #[*] rclpyを通したrosのコミュニケーションが行えるようにします．
    node = Bringme_state() #[*] ステートマシーンのノードを初期化します．
    node.execute() #[*] ステートマシーンを実行します．


# 音声認識関連の状態
class Voice(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] 音声認識関連の状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
            self,
            output_keys=['target_object', 'target_location'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger() #[*] ロガーを定義します．
        
        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'voice/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('音声認識の状態を開始します') #[*] 音声認識関連の状態が実行されたら，ログを残します．

        self.req.command = 'start' #[*] 音声認識関連のサービスを実行するようにリクエストを送信します．
        result = self.send_request()

        if result: #[*] 音声認識で認識結果が得られた場合
            userdata.target_object = self.target_object #[*] 認識した物体と場所の名前を，他の状態に値を渡す際の変数に入力します．
            userdata.target_location = self.target_location

            return 'succeeded' #[*] 成功を返します．
        else:
            return 'failed' #[*] 失敗を返します．

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] サーバと通信を行うためのクライアントを作成します．

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer != '': #[*] レスポンスのanswer変数に文字が入っている場合
            self.target_object = 'cup'  #[*] クラス内変数に文字を代入します．find_object_name(response.answer)
            self.target_location = 'kitchen'  #[*] クラス内変数に文字を代入します． find_location_name(response.answer)

            return True
        else:
            return False


# ナビゲーションの状態
class Navigation(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] ナビゲーションの状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
            self,
            input_keys=['target_location'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger() #[*] ロガーを定義します．
        
        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'navigation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('ナビゲーションの状態を開始します') #[*] ナビゲーションの状態が実行されたら，ログを残します．

        self.req.command = userdata.target_location #[*] ナビゲーションのサービスを実行するようにリクエストを送信します．
        result = self.send_request()

        if result: #[*] 移動先に到達した場合
            return 'succeeded' #[*] 成功を返します．
        else:
            return 'failed' #[*] 失敗を返します．

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] サーバと通信を行うためのクライアントを作成します．

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'reached': #[*] レスポンスのanswer変数に reached が入っている場合
            return True
        else:
            return False


# ビジョンの状態
class Vision(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] ビジョンの状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
            self,
            input_keys=['target_object'],
            output_keys=['target_object_pos'],
            outcomes=['succeeded', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger() #[*] ロガーを定義します．

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(StringCommand, 'vision/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('ビジョンの状態を開始します') #[*] ビジョンの状態が実行されたら，ログを残します．

        self.req.command = userdata.target_object #[*] 物体認識のサービスを実行するようにリクエストを送信します．
        result = self.send_request()
        userdata.target_object_pos = [0.12, -0.03, 0.4]   #[*] 物体認識が成功したとして，物体の位置を代入しています．単位は[m]

        if result: #[*] 物体を認識した場合
            return 'succeeded'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] サーバと通信を行うためのクライアントを作成します．

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'detected': #[*] レスポンスのdetected変数に detected が入っている場合
            return True #[*] 成功を返します．
        else:
            return False #[*] 失敗を返します．


# マニピュレーションの状態
class Manipulation(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] マニピュレーションの状態における結果と，他の状態に値を渡す際の名前を事前に定義します．
            self,
            input_keys=['target_object_pos'],
            outcomes=['exit', 'failed'])

        # Nodeを作成
        self.node = node
        self.logger = self.node.get_logger() #[*] ロガーを定義します．

        # サービスにおけるクライアントを作成
        self.cli = self.node.create_client(
            StringCommand, 'manipulation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('サービスへの接続待ちです・・・')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('マニピュレーションの状態を開始します') #[*] マニピュレーションの状態が実行されたら，ログを残します．

        target_object_pos = userdata.target_object_pos #[*] アームを伸ばす先の座標を代入します．
        print(f'{target_object_pos}')

        self.req.command = 'start' #[*] マニピュレーションのサービスを実行するようにリクエストを送信します．
        result = self.send_request()

        if result: #[*] アームのリーチングが完了した場合
            return 'exit'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] サーバと通信を行うためのクライアントを作成します．

        # サービスを動作させる処理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'reached': #[*] レスポンスの変数に reached が入っている場合
            return True #[*] 成功を返します．
        else:
            return False #[*] 失敗を返します．
