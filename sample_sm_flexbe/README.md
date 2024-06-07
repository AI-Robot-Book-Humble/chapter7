# sample_sm_flexbe

7.3節のサンプルプログラム  
FlexBEによる二状態のステートマシンのプログラム


## 実行

1. FlexBE Appを実行します．
  ```
  ros2 launch flexbe_app flexbe_full.launch.py
  ```

> [!NOTE]
> `FlexBe App`が起動されない場合は，`nwjs`がインストールされていない可能性があります．
その際，`ros2 run flexbe_app nwjs_install`を実行してください．

2. FlexBE Appの上部にある`Load Behavior`を押して，右側に現れるBehavior一覧から`Sample Behavior`を選択してください．

3. その後，FlexBe Appの上部の`Runtime Control`を押し，`Start Execution`でステートマシンを開始させます．

> [!NOTE]
> `init_counter`はユーザーが食べたスナックの数のことを表します．その値を自由に変えられます．

5. `Behavior Feedback`にステートマシンの結果を確認できます．


## Statesの一覧

* [search_state.py](sample_sm_flexbe_states/sample_sm_flexbe_states/search_state.py):
  * スナックを探す状態の実装

* [eat_state.py](sample_sm_flexbe_states/sample_sm_flexbe_states/eat_state.py):
  * 見つけたスナックを食べる状態の実装 


## Behaviorsの一覧

* [sample_behavior_sm.py](sample_sm_flexbe_behaviors/sample_sm_flexbe_behaviors/sample_behavior_sm.py):
  * 食べ物の探索と食事の状態を含めたSampleタスクのためのステートマシン
