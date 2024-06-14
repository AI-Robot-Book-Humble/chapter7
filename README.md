# 第７章　プランニング
## 概要

ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第７章のサンプルプログラムと補足情報などを掲載しています．

> [!IMPORTANT]
> `bringme_sm_smach`・`pseudo_node_service`・`sample_sm_smach`というリポジトリをサポートしません．
そちらのソースコードについては，[AI Robot Book 題7章　プラニング](https://github.com/AI-Robot-Book/chapter7)の方にIssueを立ててください．


## インストール方法

本リポジトリをインストールするための手順を説明します．

### `Smach`や`FlexBE`のための環境構築

1. 必要なROS関連のパッケージをインストールします．
   ```bash
   $ sudo apt-get update
   $ sudo apt-get install -y \
      ros-humble-smach \
      ros-humble-executive-smach \
      ros-humble-flexbe-core \
      ros-humble-flexbe-widget \
      ros-humle-flexbe-behavior-engine
   ```

3. 初めて`FlexBE`を利用する場合は、`flexbe_app`をダウンロードします．
   ```bash
   $ cd ~/airobot_ws/src/
   $ git clone -b humble https://github.com/FlexBE/flexbe_app.git
   ```

4. ダウンロードしたリポジトリをコンパイルします．
   ```bash
   $ cd ~/airobot_ws
   $ colcon build --symlink-install
   $ source install/setup.bash
   ```

5. 最後に，FlexBEのGUIのため，必要な依存パッケージをダウンロードします．
   ```bash
   $ ros2 run flexbe_app nwjs_install
   ```

### 本リポジトリのセットアップ

1. 本リポジトリをダウンロードします．
   ```bash
   $ cd ~/airobot_ws/src/
   $ git clone https://github.com/AI-Robot-Book-Humble/chapter7
   ```
> [!IMPORTANT]
> `airobot_interfaces`にあるActionファイルを使用するため，そちらのリポジトリーもcloneしてください．

2. ダウンロードしたリポジトリをコンパイルします．
   ```bash
   $ cd ~/airobot_ws
   $ colcon build --symlink-install
   $ source install/setup.bash
   ```


## Behaviorsの作成方法

1. `src`のフォルダーに移動します．
  ``` bash
  $ cd ~/airobot_ws/src
  ```

2. Behaviorsのためのパッケージを作成します．
  ``` bash
  # 以下の`respository_name`を作成したパッケージの名前に書き換えてください
  $ ros2 run flexbe_widget create_repo respository_name
  ```

3. 作成されたパッケージをコンパイルします．
  ``` bash
  $ cd ~/airobot_ws/
  $ colcon build --symlink-install
  $ source ~/airobot_ws/install/setup.bash
  ```

4. `FlexBE App`を実行します．
  ``` bash
  $ ros2 launch flexbe_app flexbe_full.launch
  ```

> [!NOTE]
> `FlexBe App`が起動されない場合は，`nwjs`がインストールされていない可能性があります．
その際，`ros2 run flexbe_app nwjs_install`を実行してください．

5. `Behavior Dashboard`が表示されます．そこで，`Load Behavior`を押し，右側に表示されるBehavior一覧の中から作成したパッケージの名前を選択します．

6. 必要に応じて，`Behavior Parameters`・`Private Configuration`・`State Machine Userdata`・`State Machine Interface`に変数値を定義します．

7. 次に，`Statemachine Editor`に移動して，`Add State`という機能を利用し，事前に作成した状態を挿入します．その際に，右側に表示されるState一覧から必要な状態を選択し，`Name`に状態の名前を記入します．`Add`を押すと，ステートマシンにその状態が表示されます．

8. 入力された状態に「●」と接続します．また，その状態の可能な出録結果に応じて，次の状態か終了の「◎」と接続させます．

9. ステートマシンの構造が終わりましたら，問題がないかを`Check Behavior`で確認します．Logの結果に応じて，修正してください．

10. 最後に問題がなければ，`Save Behavior`を押し，Behaviorを保存します．


## ディレクトリ構成

- **[bringme_sm_flexbe](bringme_sm_flexbe):** Bring meタスクのステートマシンのサンプルプログラム (FlexBE版)
- **[bringme_sm_smach](bringme_sm_smach):** Bring meタスクのステートマシンのサンプルプログラム (Smach版)
- **[pseudo_node_action](pseudo_node_action):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム (Action版)
- **[pseudo_node_service](pseudo_node_service):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム (Service版)
- **[sample_sm_flexbe](sample_sm_flexbe):** 二状態のステートマシンのサンプルプログラム (FlexBE版)
- **[sample_sm_smach](sample_sm_smach):** 二状態のステートマシンのサンプルプログラム (Smach版)
