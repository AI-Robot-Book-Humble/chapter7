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

"""Search state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

from time import sleep


class SearchState(EventState):
    """
    SearchStateという状態はスイーツを探すことを目標としています．
    ユーザーがこれまでどれぐらい食べたかによって，食べるか食べないかを判定します．

    出力
    <= succeeded       スイーツが見つけたら，検出成功するという結果を出力します
    <= finished        スイーツでお腹いっぱいになったら，検索終了の結果を出力します
    <= failed          何らかの問題で，検索を失敗した場合，失敗したという結果を出力します

    Userdata
    ># eat_counter  int ユーザーがこれまでまで食べたスイーツの数       (int型) (Input)
    ># max_eat      int ユーザーがお腹いっぱいになるまでのスイーツの数 (int型) (Input)
    """

    def __init__(self):
        """状態の結果，入力キーを定義します．"""
        super().__init__(outcomes=['succeeded', 'finished'],
                         input_keys=['eat_counter', 'max_eat'])

    def execute(self, userdata):
        # search処理を開始します
        sleep(1)
        Logger.loginfo('スイーツを探索しています') # 探索の状態にいることをログに残します

        if userdata.eat_counter < userdata.max_eat:
            Logger.loginfo('スイーツを見つけました！') # 探索の状態へ訪れた回数が3回未満の場合

            return 'succeeded' # 'succeeded'という結果を返します
        else:
            Logger.loginfo('もうお腹いっぱいです・・・') # 探索の状態へ訪れた回数が3回になった場合

            return 'finished' # 'finished'という結果を返します
