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


class SearchState(EventState):
    """
    SearchStateという状態はスナックを探すことを目標としています．
    ユーザーがこれまでどれぐらい食べたかによって，食べるか食べないかを判定します．

    出力
    <= succeeded       スナックが見つけたら，検出成功するという結果を出力します
    <= finished        スナックでお腹いっぱいになったら，検索終了の結果を出力します
    <= failed          何らかの問題で，検索を失敗した場合，失敗したという結果を出力します

    Userdata
    ># eat_counter  int ユーザーがこれまでまで食べたスナックの数(int型) (Input)
    """

    def __init__(self):
        """状態の結果，入力キーを定義します．"""
        super().__init__(outcomes=['succeeded', 'finished', 'failed'],
                         input_keys=['eat_counter'])
        self._error = False
        self._return = None

    def execute(self, userdata):
        # _errorがあるかを確認します
        if self._error:
            return 'failed' # 'failed'という結果を返します

        # 遷移がブロックされた場合には前の戻り値を戻します
        if self._return is not None:
            return self._return

        # 検索を始めます
        Logger.loginfo('スイーツを探索しています') # 探索の状態にいることをログに残します
        if userdata.eat_counter < 3:
            Logger.loginfo('スイーツを見つけました！') # 探索の状態へ訪れた回数が3回未満の場合

            return 'succeeded' # 'succeeded'という結果を返します
        else:
            Logger.loginfo('スイーツを見つけたけど，もうお腹いっぱいです・・・') #[*] 探索の状態へ訪れた回数が3回になった場合

            return 'finished' # 'finished'という結果を返します

        return None  # ステートが終わっていなければ，Noneを戻します

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
