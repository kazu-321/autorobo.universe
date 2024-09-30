# autorobo.universe

2024NHK高専ロボコン　茨城高専Bチーム

ごじゃっペサーカスR1の制御です。

# important!
ライセンス整備がまだです！

## ビルド
```bash
$ # cd to your ws
$ cd src
$ git clone https://github.com/kazu-321/autorobo.universe
$ cd ../
$ colcon build
$ source install/setup.bash
```

## 実行
terminal 0
```bash
ros2 launch autorobo_launch sim.launch.xml
```

terminal 1
```bash
ros2 run autorobo_teleop teleop_key
```

terminal 1の操作方法
- cキー

        - コンティニュー
        - ロボコンのルールに基づいた遠隔非常停止解除用
- pキー

        - ポーズ
        - サイドコンティニュー(c)すればロボットを動かせる
- waxdキー

        - 4方向移動
        - nav offの状態じゃないと動けない
- sキー

        - ユーザー入力による移動ストップ
- nキー

        - 自動運転のon off切り替え用
        - 最初はon
- oキー

        - 射出許可など自動運転時に安全のためにあるボタン
        - コンティニュー&&自動運転on　ならパスが生成されたりする

## 未検証
- pynputを使用したバックグラウンドキー入力
- rosbag関係のlaunch
- 実際に使用した慣性モデル付きのシミュレーター