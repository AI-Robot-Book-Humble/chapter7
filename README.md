# 第７章　プランニング
## 概要
ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第７章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成

- **[bringme_sm](https://github.com/AI-Robot-Book/chapter7/tree/master/bringme_sm):** Bring meタスクのステートマシンのサンプルプログラム
- **[pseudo_node](https://github.com/AI-Robot-Book/chapter7/tree/master/pseudo_node):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム
- **[sample_sm](https://github.com/AI-Robot-Book/chapter7/tree/master/sample_sm):** 二状態のステートマシンのサンプルプログラム
   
## 補足情報
- Smachがpythonで使えるか確認する手順
   - 以下のコマンドを実行
   ```
   python3 -c "import smach"
   ```
   - なにも出力されなければsmachはインストールされています
   - ModuleNotFoundError: No module named 'smach'と出力され場合は次のコマンドを実行してください．
   ```
   pip3 install smach
   ```
