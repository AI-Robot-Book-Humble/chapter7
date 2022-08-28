# sample_sm
## 概要
7.3節のサンプルプログラム  
二状態のステートマシンのプログラム

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
  - 端末を開いて二状態のステートマシーンを実行します．
  ```
  ros2 run sample_sm sample_sm
  ```

## ヘルプ

## 著者
萩原　良信

## 履歴
バグフィクスやチェンジログ
- 2022-08-23: READMEの修正
- 2022-03-15: 初期版

## ライセンス
Copyright (c) 2022, Yoshinobu Hagiwara and Masaki Ito
All rights reserved.
This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
