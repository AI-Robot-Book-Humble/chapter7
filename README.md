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
  ```console
  $ sudo apt-get update
  $ sudo apt-get install -y \
      ros-humble-smach \
      ros-humble-executive-smach
  ```

3. 初めて`FlexBE`を利用する場合は、`FlexBE Behavior Engine`と`FlexBE WebUI`をダウンロードします．
  ```console
  $ cd ~/airobot_ws/src/
  $ git clone -b 4.0.0 https://github.com/FlexBE/flexbe_behavior_engine.git
  $ git clone https://github.com/AI-Robot-Book-Humble/flexbe_webui.git
  ```

4. ダウンロードしたリポジトリをコンパイルします．
  ```console
  $ cd ~/airobot_ws/
  $ rosdep update
  $ rosdep install --from-paths src --ignore-src
  $ colcon build
  $ source install/setup.bash
  ```

5. 最後に，`rosdep install`より`FlexBE WebUI`のための依存パッケージがインストールされなかった場合，以下のコマンドでダウンロードします．
   ```console
   $ cd ~/airobot_ws/src/flexbe_webui/
   $ pip3 install -r requires.txt
   ```


### 本リポジトリのセットアップ

1. 本リポジトリをダウンロードします．
  ```console
  $ cd ~/airobot_ws/src/
  $ git clone https://github.com/AI-Robot-Book-Humble/chapter7.git
  ```
> [!IMPORTANT]
> [airobot_interfaces](https://github.com/AI-Robot-Book-Humble/airobot_demo)にあるActionファイルを使用するため，そちらのリポジトリーもcloneしてください．

2. ダウンロードしたリポジトリをコンパイルします．
  ```console
  $ cd ~/airobot_ws/
  $ colcon build
  $ source install/setup.bash
  ```


## ステートマシンの作成方法

1. ワークスペースのPATHを設定します．
  ``` console
  $ echo "export WORKSPACE_ROOT=~/airobot_ws" >> ~/.bashrc
  $ source ~/.bashrc
  ```

2. ワークスペースにある`src`のフォルダーに移動します．
  ``` console
  $ cd ~/airobot_ws/src/
  ```

3. Behaviorsのためのパッケージを作成します．
  ``` console
  $ ros2 run flexbe_widget create_repo hello_world
  ```
> [!NOTE]
> `hello_world`という部分を作成したいリポジトリ名に書き換えても構いません．

4. 質問に対して，「**no**」と回答します．
  ```console
  Initializing new behaviors repo hello_world_behaviors ...

  (2/5) Fetching project structure...
  Cloning into 'hello_world_behaviors'...
  remote: Enumerating objects: 156, done.
  remote: Counting objects: 100% (156/156), done.
  remote: Compressing objects: 100% (84/84), done.
  remote: Total 156 (delta 62), reused 149 (delta 55), pack-reused 0
  Receiving objects: 100% (156/156), 32.57 KiB | 4.07 MiB/s, done.
  Resolving deltas: 100% (62/62), done.
  Set up for ROS 2 development ...
  Already on 'ros2-devel'
  Your branch is up to date with 'origin/ros2-devel'.

  (3/5) Configuring project template...
  mv: 'PROJECT_behaviorshello_world_behaviors' の後に宛先のファイルオペランドがありません
  詳しくは 'mv --help' を実行して下さい。

  (4/5) Removing the original git repository...
  (5/5) Do you want to initialize a new Git repository for this project? (yes/no) no
  ```

4. 作成されたパッケージをコンパイルします．
  ``` console
  $ cd ~/airobot_ws/
  $ colcon build
  $ source install/setup.bash
  ```

5. `FlexBE WebUI`を実行します．
  ``` console
  $ ros2 launch flexbe_webui flexbe_full.launch.py
  ```

> [!NOTE]
> `FlexBe WebUI`が起動されない場合は，依存パッケージがインストールされていない可能性があります．
その際，`flexbe_webui`のフォルダーの中で，`pip3 install -r requires.txt`を実行してください．

6. `Behavior Dashboard`が表示されます．
![](docs/hello_world/01_behavior_dashboard.png)

7. `Load Behavior`を押し，右側にBehavior一覧が表示されます．
![](docs/hello_world/02_load_behavior.png)

8. その中から，`Example Behavior`というBehaviorを選択します．
![](docs/hello_world/03_loaded_behavior.png)

9. `Statemachine Editor`に移動して，ステートマシンの状態を確認します．
![](docs/hello_world/04_statemachine_editor.png)

10. `Runtime Control`に移動して，ステートマシンを実行します．
そのために，まず`Waiting Time`という待機時間を表す初期値を設定します．
![](docs/hello_world/05_runtime_control.png)

11. 次に，`Start Execution`を押して，状態が開始されます．

| Printステート | Waitステート |
| --- | --- |
| ![](docs/hello_world/06_runtime_control_running_1.png) | ![](docs/hello_world/07_runtime_control_running_2.png) |

> [!NOTE]
> `Printステート`から`Waitステート`へ進行させるために，`Autonomy`を`Low`から`Full`に変更してください．

12. 実行ターミナルの結果の一例．
  ```console
  [00:28:31] Onboard engine is ready.
  [00:28:35] --> Mirror - received updated structure with checksum id = 10094639919
  [00:28:35] Activate mirror for behavior id = 10094639919 ...
  [00:28:35] Executing mirror ...
  [00:28:35] --> Preparing new behavior...
  [00:28:35] Onboard Behavior Engine starting [Example Behavior : 10094639919]
  [00:28:35] Hello World!
  [00:28:39] PreemptableStateMachine 'Example Behavior' spin() - done with outcome=finished
  [00:28:39] No behavior active.
  [00:28:39] [92m--- Behavior Mirror ready! ---[00m
  [00:28:39] Onboard engine is ready.
  ```


## ディレクトリ構成

- **[sample_sm_flexbe](sample_sm_flexbe):** 二状態のステートマシンのサンプルプログラム (FlexBE版)
- **[bringme_sm_flexbe](bringme_sm_flexbe):** Bring meタスクのステートマシンのサンプルプログラム (FlexBE版)
- **[pseudo_node_action](pseudo_node_action):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム (Action版)
- **[(アーカイブ) sample_sm_smach](sample_sm_smach):** 二状態のステートマシンのサンプルプログラム (Smach版)
- **[(アーカイブ) bringme_sm_smach](bringme_sm_smach):** Bring meタスクのステートマシンのサンプルプログラム (Smach版)
- **[(アーカイブ) pseudo_node_service](pseudo_node_service):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム (Service版)
