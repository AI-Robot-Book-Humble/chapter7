# bringme_sm
## 概要
7.4節のサンプルプログラム  
Bring meタスクのためのステートマシンのプログラム

## インストール
  - 以下のコマンドでワークスペースのsrcに移動してexecutive smachとchapter7のサンプルプログラムをGitHubからクローンします．
  ```
  cd ˜/airobot_ws/src
  git clone https://github.com/jeffrey870517/executive_smach
  git clone https://github.com/AI-Robot-Book/chapter7
  ```
  - 以下のコマンドでワークスペースのルートに戻り，ビルドします．
  ```
  cd ˜/airobot_ws
  colcon build
  ```
  - ビルドが完了したらsetup.bashを実行します．
  ```
  source install/setup.bash
  ```

## 実行
  - 端末を開いてステートマシーンを実行します．
  ```
  ros2 launch bringme_sm bringme.launch.py
  ```

## ヘルプ

## 著者
萩原　良信

## 履歴
- 2022-08-28: 初期版

## ライセンス
Copyright (c) 2022, Yoshinobu Hagiwara and Masaki Ito
All rights reserved.
This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
