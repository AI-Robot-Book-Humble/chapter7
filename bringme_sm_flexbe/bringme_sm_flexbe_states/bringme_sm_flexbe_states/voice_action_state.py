#!/usr/bin/env python

# Copyright 2023 Christopher Newport University
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

"""Voice Action FlexBE State."""

# import math

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from airobot_interfaces.action import StringCommand

class VoiceActionState(EventState):
    """
    Action通信による音声認識を起動し，その結果をuserdata.textに代入する

    Execution notes:
        こちらの状態を実行するために，必要なAction Serverを起動してください：
        ros2 run pseudo_node_action voice_node

        実行可能なAction一覧を表示させるために，以下のコマンドを実行してください：
        ros2 action list

    Parameters
    -- timeout             最大許容時間 (seconds)
    -- action_topic        音声認識のアクション名

    Outputs
    <= done                Only a few dishes have been cleaned.
    <= failed              何らかの理由で失敗
    <= canceled            ユーザーからのキャンセルリクエストが送信された
    <= timeout             The action has timed out.

    User data
    ># time     string     音声認識の実行時間 (秒数) (Input)
    #> text     string     音声認識の結果 (string型) (Output)

    """

    def __init__(self, timeout, action_topic="ps_voice/command"):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed', 'canceled', 'timeout'],
                         input_keys=['time'],
                         output_keys=['text'])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        ProxyActionClient.initialize(VoiceActionState._node)

        self._client = ProxyActionClient({self._topic: StringCommand},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._start_time = None

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)  # The delta result value is not useful here
            userdata.text = result.answer

            if userdata.text == 'failed':
                Logger.logwarn('音声認識が失敗しました')
                self._return = 'failed'
                return self._return
            else:
                Logger.loginfo(f'音声認識の結果: {userdata.text}')
                self._return = 'done'
                return self._return

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Checking for timeout after we check for goal response
            Logger.loginfo('Timeout')
            self._return = 'timeout'
            return 'timeout'

        # If the action has not yet finished, no outcome will be returned and the state stays active.
        return None

    def on_enter(self, userdata):

        # make sure to reset the error state since a previous state execution might have failed
        self._error = False
        self._return = None

        if 'time' not in userdata:
            self._error = True
            Logger.logwarn("VoiceActionState requires userdata.time key!")
            return

        # Recording the start time to set rotation duration output
        self._start_time = self._node.get_clock().now()

        goal = StringCommand.Goal()

        if isinstance(userdata.time, (str)):
            goal.command = str(userdata.time)  # convert to radians
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects an string.", type(userdata.time).__name__)

        # Send the goal.
        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the Time command:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
