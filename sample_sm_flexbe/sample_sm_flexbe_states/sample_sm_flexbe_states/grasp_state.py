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

"""Grasp state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

import random
from time import sleep


class GraspState(EventState):
    """
    GraspStateという状態はスイーツを把持することを目標としています．

    出力
    <= succeeded       見つけたスイーツに対して，把持成功するという結果を出力します
    <= failed          何らかの問題で，把持を失敗した場合，失敗したという結果を出力します
    """

    def __init__(self):
        """状態の結果，入力キーを定義します．"""
        super().__init__(outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        # grasp処理を開始します
        sleep(1) # 処理を可視化するために，1秒停止します
        Logger.loginfo('スイーツを把持してみます') # 探索の状態にいることをログに残します
        
        prob = random.random() # [0,1]の値を抽出します
        if 0.5 > prob:
            Logger.loginfo('スイーツを把持できました！') # 状態Graspが成功した場合

            return 'succeeded' # 'succeeded'という結果を返します
        else:
            Logger.loginfo('スイーツを把持できなかった・・・もう一度やってみます！') # 状態Graspが失敗した場合

            return 'failed' # 'finished'という結果を返します
