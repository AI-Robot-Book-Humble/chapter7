# bringme_sm_flexbe

7.4節のサンプルプログラム  
Bring meタスクのためのFlexBEによるステートマシンのプログラム


## 実行

1. Bringmeタスクのための必要なActionServerを実行するために，[bringme_nodes.launch.py](../pseudo_node_action/launch/bringme_nodes.launch.py)というlaunchファイルを実行します．
  ```
  ros2 launch pseudo_node_action bringme.launch.py
  ```

2. FlexBE Appを実行します．
  ```
  ros2 launch flexbe_app flexbe_full.launch.py
  ```

> [!NOTE]
> `FlexBe App`が起動されない場合は，`nwjs`がインストールされていない可能性があります．
その際，`ros2 run flexbe_app nwjs_install`を実行してください．

3. FlexBE Appの上部にある`Load Behavior`を押して，右側に現れるBehavior一覧から`Bringme Action Behavior`を選択してください．

4. その後，FlexBe Appの上部の`Runtime Control`を押し，`Start Execution`でステートマシンを開始させます．

> [!NOTE]
> `init_time`は音声認識の起動時間のことを表します．その値を自由に変えられます．

5. `Behavior Feedback`にステートマシンの結果を確認できます．


## Statesの一覧

* [voice_action_state.py](bringme_sm_flexbe_states/bringme_sm_flexbe_states/voice_action_state.py):
  * 音声認識の状態の実装

* [navigation_action_state.py](bringme_sm_flexbe_states/bringme_sm_flexbe_states/navigation_action_state.py):
  * ナビゲーションの状態の実装

* [vision_action_state.py](bringme_sm_flexbe_states/bringme_sm_flexbe_states/vision_action_state.py):
  * 物体認識の状態の実装

* [manipulation_action_state.py](bringme_sm_flexbe_states/bringme_sm_flexbe_states/manipulation_action_state.py):
  * 物体把持の状態の実装

## Behaviorsの一覧

* [bringme_action_behavior_sm.py](bringme_sm_flexbe_behaviors/bringme_sm_flexbe_behaviors/bringme_action_behavior_sm.py):
  * 音声認識・ナビゲーション・物体認識・物体把持の状態を含めたBringmeタスクのためのステートマシン 


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