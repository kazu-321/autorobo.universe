# autorobo.universe

2024NHK高専ロボコン　茨城高専Bチーム

ごじゃっペサーカスR1の制御です。

## ビルド
```
colcon build
```

## ファイル構成
```
autorobo.universe
├── behavior / autorobo_behavior
│       └── behavior_node
│
├── common
│   ├── autorobo_teleop
│   │       ├── background_key
│   │       └── teleop_key
│   │
│   └── autorobo_visualization
│           ├── visualize_lines_node
│           └── visualize_stl_node
│
├── control / autorobo_control
│       └── pure_pursuit_node
│
├── launch / autorobo_launch
│   ├── launch
│   │   ├── common
│   │   │   ├── bridge.launch.xml
│   │   │   ├── marker.launch.xml
│   │   │   ├── start_rviz.launch.xml
│   │   │   ├── tf.launch.xml
│   │   │   └── visualization
│   │   │       ├── field_area.launch.xml
│   │   │       ├── movable_area.launch.xml
│   │   │       ├── robot_area.launch.xml
│   │   │       ├── start_left_area.launch.xml
│   │   │       ├── start_right_area.launch.xml
│   │   │       ├── visualize_field.launch.xml
│   │   │       └── visualize_robot.launch.xml
│   │   ├── localization.launch.xml
│   │   ├── planning.launch.xml
│   │   ├── sim.launch.xml
│   │   └── ydlidar.launch.py
│   └── model3d
│       ├── field.stl
│       └── robot.stl
│
├── localization / autorobo_localization
│       └── ransac_node.cpp
│
├── message / autorobo_msgs
│       └── Twistring.msg
│
├── planning / autorobo_planner
│       └── planner_node.cpp
│
└── simulation / autorobo_simulation
        └── simulation_node.cpp
```