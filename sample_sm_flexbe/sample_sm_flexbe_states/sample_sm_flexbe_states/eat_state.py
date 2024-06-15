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

"""Eat state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

from time import sleep


class EatState(EventState):
    """
    EatStateという状態は前の状態に見つけたスナックを食べることを目標とします．
    ユーザーがこれまで食べたスナックの数を考慮せず，ランダムに食べるか食べないかを判定します．

    出力
    <= succeeded       Eat状態を終了したことを出力します

    Userdata
    ># eat_counter  int ユーザーがこれまでまで食べたスナックの数 (int型) (Input)
    #> eat_counter  int 食べたスナックの数を更新し，出力します (int型) (Output)
    """

    def __init__(self):
        """状態の結果，入力キーを定義します．"""
        super().__init__(outcomes=['succeeded'],
                         input_keys=['eat_counter'],
                         output_keys=['eat_counter'])

    def execute(self, userdata):
        # eat処理を開始します
        sleep(1)
        Logger.loginfo('スイーツを1個食べます！') # 食事をしたことをログに残します
        userdata.eat_counter += 1 # eat_counterを更新します
        Logger.loginfo('現時点では、スイーツを{}個食べました！'.format(userdata.eat_counter)) # 食べたスナックの数をログに残します

        return 'succeeded' # 'succeeded'という結果を返します
