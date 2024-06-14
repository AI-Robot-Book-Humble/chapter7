# bringme_sm_flexbe

7.4節のサンプルプログラム  
Bring meタスクのためのFlexBEによるステートマシンのプログラム


## 実行

1. Bringmeタスクのための必要なActionServerを実行するために，[bringme_nodes.launch.py](../pseudo_node_action/launch/bringme_nodes.launch.py)というlaunchファイルを実行します．
  ```console
  $ ros2 launch pseudo_node_action bringme.launch.py
  ```

2. FlexBE Appを実行します．
  ```console
  $ ros2 launch flexbe_app flexbe_full.launch.py
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
