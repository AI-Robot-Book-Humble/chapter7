# 第７章　プランニング
## 概要

ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第７章のサンプルプログラムと補足情報などを掲載しています．

> [!IMPORTANT]
> `bringme_sm_smach`・`pseudo_node_service`・`sample_sm_smach`というServiceやSmachの機能を用いたリポジトリーをサポートしません．
そちらのソースコードについてご質問があれば，[AI Robot Book 題7章　プラニング](https://github.com/AI-Robot-Book/chapter7)にIssueを立ててください．


## インストール方法

本リポジトリをインストールするための手順を説明します．

1. 本リポジトリーをダウンロードします．
   ```bash
   $ cd ~/airobot_ws/src/
   $ git clone https://github.com/AI-Robot-Book-Humble/chapter7
   ```
> [!IMPORTANT]
> `airobot_interfaces`にあるActionファイルを使用するため，そちらのリポジトリーもcloneしてください．

2. 必要なROS関連のパッケージをインストールします．
   ```bash
   $ sudo apt-get update
   $ sudo apt-get install -y \
      ros-${ROS_DISTRO}-smach \
      ros-${ROS_DISTRO}-executive-smach \
      ros-${ROS_DISTRO}-flexbe-core \
      ros-${ROS_DISTRO}-flexbe-widget \
      ros-${ROS_DISTRO}-flexbe-behavior-engine
   ```
> [!NOTE]
> ROSはすでにインストールされている場合は， `{ROS_DISTRO}` という部分は自動的に `humble` に書き換えられますので，ご安心ください.

3. 初めて`FlexBE`を利用する場合は、`flexbe_app`をダウンロードします．
   ```bash
   $ cd ~/airobot_ws/src/
   $ git clone -b humble https://github.com/FlexBE/flexbe_app.git
   ```

4. ダウンロードしたリポジトリをコンパイルします
   ```bash
   $ cd ~/airobot_ws
   $ colcon build --symlink-install
   $ source ~/airobot_ws/install/setup.bash
   ```

5. 最後に，`flex_app`を初めて使用する場合，必要な依存パッケージをダウンロードします．
   ```bash
   $ ros2 run flexbe_app nwjs_install
   ```


## ディレクトリ構成

- **[bringme_sm_flexbe](bringme_sm_flexbe):** Bring meタスクのステートマシンのサンプルプログラム (FlexBE版)
- **[bringme_sm_smach](bringme_sm_smach):** Bring meタスクのステートマシンのサンプルプログラム (Smach版)
- **[pseudo_node_action](pseudo_node_action):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム (Action版)
- **[pseudo_node_service](pseudo_node_service):** Bring meタスクにおける，音声，ナビゲーション，ビジョン，マニピュレーションの疑似ノードのサンプルプログラム (Service版)
- **[sample_sm_flexbe](sample_sm_flexbe):** 二状態のステートマシンのサンプルプログラム (FlexBE版)
- **[sample_sm_smach](sample_sm_smach):** 二状態のステートマシンのサンプルプログラム (FlexBE版)
