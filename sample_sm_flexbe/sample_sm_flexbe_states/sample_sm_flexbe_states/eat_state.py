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

import random


class EatState(EventState):
    """
    This example lets the behavior wait until the given target_time has passed since
    entering the state.

    List labeled outcomes using the double arrow notation (must match constructor)
    <= done            Given time has passed.

    User data
    ># eat_counter  int 目的地への移動の実行時間 (int型) (Input)
    #> eat_counter  int 移動の結果 (int型) (Output)
    """

    def __init__(self):
        """Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments."""
        super().__init__(outcomes=['done'],
                         input_keys=['eat_counter'],
                         output_keys=['eat_counter'])
        # Retain return value in case the outcome is blocked by operator
        self._error = False
        self._return = None

    def execute(self, userdata):
        if self._return is not None:
            return self._return

        prob = random.random()

        if 0.5 > prob:
            Logger.loginfo('スイーツを1個食べます！') #[*] 食事の状態にいることをログに残します．
            userdata.eat_counter += 1

        else:
            Logger.loginfo('今回は我慢しようか。。。') #[*] 食事の状態にいることをログに残します．

        return 'done' #[*] 'done'という結果を返します．

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
