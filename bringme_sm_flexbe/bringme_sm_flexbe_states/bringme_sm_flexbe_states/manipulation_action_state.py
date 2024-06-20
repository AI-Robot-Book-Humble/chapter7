#!/usr/bin/env python

# Copyright 2024 Keith Valentin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Manipulation Action FlexBE State."""

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from airobot_interfaces.action import StringCommand


class ManipulationActionState(EventState):
    """
    Action通信による物体把持を起動し，その結果をuserdata.textに代入します

    起動方法:
        こちらの状態を実行するために，必要なAction Serverを起動してください
        $ ros2 run pseudo_node_action manipulation_node

        実行可能なAction一覧を表示させるために，以下のコマンドを実行してください
        $ ros2 action list

    パラメーター
    -- timeout             最大許容時間 (seconds)
    -- action_topic        物体把持のアクション名

    出力
    <= done                物体把持が成功した場合
    <= failed              何らかの理由で失敗した場合
    <= canceled            ユーザーからのキャンセルリクエストした場合
    <= timeout             目的地への移動の最大許容時間を超過した場合

    Userdata
    ># target     string   把持の対象物体の名前 (string型) (Input)
    #> text       string   Actionによる物体把持の結果 (string型) (Output)

    """

    def __init__(self, timeout, action_topic="/ps_manipulation/command"):
        super().__init__(outcomes=['done', 'failed', 'canceled', 'timeout'],
                         input_keys=['target'],
                         output_keys=['text'])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        self._error      = False # ActionのClientからGoalの送信を失敗した場合
        self._return     = None  # オペレーターによる結果の出力を拒む場合，戻り値を保存します
        self._start_time = None  # 開始時間を初期化します

        # FlexBEのProxyActionClientを用いてActionのClient側を作成します
        ProxyActionClient.initialize(ManipulationActionState._node)
        self._client = ProxyActionClient({self._topic: StringCommand},
                                         wait_duration=0.0)

    def execute(self, userdata):
        '''
        起動中，Actionが終了したかどうかを確認し，その結果によってoutcomeを決定します
        '''

        # _errorが起きたかを確認します
        if self._error:
            return 'failed' # 'failed'という結果を返します

        # 遷移がブロックされた場合には前の戻り値を戻します
        if self._return is not None:
            return self._return

        # Actionが終了したかを確認します
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic) # Actionの結果を取得します
            userdata.text = result.answer

            if userdata.text == 'failed':
                Logger.logwarn('物体把持が失敗しました')
                self._return = 'failed'
                return self._return # 'failed'という結果を返します
            else:
                Logger.loginfo(f'物体把持の結果: {userdata.text}')
                self._return = 'done'
                return self._return # 'done'という結果を返します

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # 最大許容時間を超過したかを確認します
            Logger.loginfo('最大許容時間を経過しました')
            self._return = 'timeout'
            return self._return # 'timeout'という結果を返します

        # Actionはまだ終了していない場合，状態を終了させません
        return None

    def on_enter(self, userdata):
        # データの初期化を行います
        self._error = False
        self._return = None

        # userdataにtargetという情報があるかを確認します
        if 'target' not in userdata:
            self._error = True
            Logger.logwarn("ManipulationActionState を実行するには， userdata.target が必要です!")
            return

        # 入力された値はstring型か確認します
        if not isinstance(userdata.target, (str)):
            self._error = True
            Logger.logwarn('入力された型は %s です．string型が求められています', type(userdata.target).__name__)

        # 開始時間を記録します
        self._start_time = self._node.get_clock().now()

        # GoalをActionのServerに送信します
        goal = StringCommand.Goal()
        goal.command = str(userdata.target)

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Goalの送信が失敗しました:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        # Actionが起動していないことを確認します
        # Actionが動作していることは，オペレーターによる手動的な出力の結果が送信されたと考えられます
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('動作中のActionをキャンセルします．')
