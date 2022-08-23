# 第７章　プランニング
## 概要
ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第７章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成

- **bringme_sm:** Bring meタスクのステートマシンのサンプルプログラム
- **pseudo_node:** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム
- **sample_sm:** 二状態のステートマシンのサンプルプログラム
   
## 補足情報
- Smachがpythonで使えるかの確認手順
  - python3 -c "import smach"
  - なにも出力されなければsmachはインストールされています
  - ModuleNotFoundError: No module named 'smach' と出力され場合は次のコマンドを実行してください．
  - pip3 install smach

- 2状態のステートマシーンの実行手順（7.3節）
  - 端末を開いて2状態のステートマシーンを実行します．
    - ros2 run sample_sm sample_sm
- 物を持ってくるタスクを達成するステートマシーンの実行手順（7.4節）
  - 端末を開いてステートマシーンを実行します．
    - ros2 launch bringme_sm bringme.launch.py
