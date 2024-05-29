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

import random


class EatState(EventState):
    """
    EatStateという状態は前の状態に見つけたスナックを食べることを目標とします．
    ユーザーがこれまで食べたスナックの数を考慮せず，ランダムに食べるか食べないかを判定します．

    出力
    <= done            Eat状態を終了したことを出力します
    <= failed          何らかの問題で，検索を失敗した場合，失敗したという結果を出力します

    Userdata
    ># eat_counter  int ユーザーがこれまでまで食べたスナックの数 (int型) (Input)
    #> eat_counter  int 食べたスナックの数を更新し，出力します (int型) (Output)
    """

    def __init__(self):
        """状態の結果，入力キーを定義します．"""
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['eat_counter'],
                         output_keys=['eat_counter'])
        self._error = False
        self._return = None

    def execute(self, userdata):
        # _errorがあるかを確認します
        if self._error:
            return 'failed' # 'failed'という結果を返します

        # 遷移がブロックされた場合には前の戻り値を戻します
        if self._return is not None:
            return self._return

        prob = random.random() # [0,1]の間の値をランダムに抽出します

        if 0.5 > prob:
            Logger.loginfo('スイーツを1個食べます！') # 食事をしたことをログに残します
            userdata.eat_counter += 1 # eat_counterを更新します

        else:
            Logger.loginfo('今回は我慢しようか。。。') # 食事をスキップしたことをログに残します

        return 'done' # 'done'という結果を返します

    def on_enter(self, userdata):
        # データの初期化を行います
        self._error = False
        self._return = None

        # eat_counterというuserdataが入力されているか確認します
        if 'eat_counter' not in userdata:
            self._error = True
            Logger.logwarn('SearchStateを実行するにはuserdata.eat_counterというキーが必要です!')
            return

        # 入力された値はint型か確認します
        if not isinstance(userdata.eat_counter, (int)):
            self._error = True
            Logger.logwarn('入力された型は %s です．int型が求められています', type(userdata.eat_counter).__name__)
            return
