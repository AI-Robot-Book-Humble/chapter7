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

"""Demonstration state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

class SearchState(EventState):
    """
    This example lets the behavior wait until the given target_time has passed since
    entering the state.

    Outputs
    <= succeeded       Given time has passed.
    <= finished        Example for a failure outcome.
    <= failed          Example for a failure outcome.


    User data
    ># eat_counter  int 目的地への移動の実行時間 (int型) (Input)

    """

    def __init__(self):
        """Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments."""
        super().__init__(outcomes=['succeeded', 'finished', 'failed'],
                         input_keys=['eat_counter'])
        # Retain return value in case the outcome is blocked by operator
        self._error = False
        self._return = None

    def execute(self, userdata):
        """
        Execute this method periodically while the state is active.

        Main purpose is to check state conditions and trigger a corresponding outcome.
        If no outcome is returned, the state will stay active.
        """
        if self._return is not None:
            return self._return

        Logger.loginfo('スイーツを探索しています') #[*] 探索の状態にいることをログに残します．
        if userdata.eat_counter < 3:
            Logger.loginfo('スイーツを見つけました！') #[*] 探索の状態へ訪れた回数が3回未満の場合

            return 'succeeded' #[*] 'succeeded'という結果を返します．
        else:
            Logger.loginfo('スイーツを見つけたけど，もうお腹いっぱいです・・・') #[*] 探索の状態へ訪れた回数が3回になった場合

            return 'finished' #[*] 'finished'という結果を返します．

        return None  # This is normal behavior for state to continue executing

    def on_enter(self, userdata):
        self._return = None
        
        if 'eat_counter' not in userdata:
            self._error = True
            Logger.logwarn("SearchState requires userdata.eat_counter key!")
            return
        
        if not isinstance(userdata.eat_counter, (int)):
            self._error = True
            Logger.logwarn("Input is %s. Expects an integer.", type(userdata.eat_counter).__name__)
            return
