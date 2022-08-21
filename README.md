# ai-robot-chapter7

- 注意事項
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
