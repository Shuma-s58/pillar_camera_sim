# pillar_camera_sim — ROS 2 / RViz2 2本柱・仮想カメラシミュレーション

2本の固定された柱をRViz2で表示し、**カメラだけ**をキーボードで前後（奥行き）・左右・上下に動かすROS 2パッケージです。仮想カメラ画像には画像中心線と柱2本の中心位置を描画し、柱間隔から距離を推定します。

> **注意:** ROS 2実行環境での動作確認は未実施です。カメラ画像は幾何投影で生成したもので、Gazebo等の物理シミュレーションや実カメラの画像認識ではありません。

## 1. 必要な環境

- Ubuntu + ROS 2（Humble / Jazzy / Kiltedを想定）
- `colcon`、`ament_python`、`rclpy`、`launch_ros`
- `rviz2`、`sensor_msgs`、`geometry_msgs`、`visualization_msgs`、`std_msgs`、`tf2_ros`
- Python `numpy`、`cv2`（OpenCV）

ROS 2を導入済みの場合の追加インストール例:

```bash
source /opt/ros/<ディストリビューション名>/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-numpy python3-opencv \
  ros-${ROS_DISTRO}-rviz2
```

`<ディストリビューション名>` は `humble` や `jazzy` などに置き換えてください。

## 2. GitHubから配置する

### A. GitHubリポジトリ直下に `pillar_camera_sim/` がある場合

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <リポジトリURL>
```

この場合、`~/ros2_ws/src/<リポジトリ名>/pillar_camera_sim/package.xml` が存在することを確認します。`colcon` は通常 `src` 以下を再帰的に探索します。

### B. ZIPから配置する場合

ZIP内の `src/pillar_camera_sim/` フォルダを、既存ワークスペースの `~/ros2_ws/src/` にコピーします。

最終的な配置例:

```text
~/ros2_ws/
└── src/
    ├── 既存パッケージ/
    └── pillar_camera_sim/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── README.md
        ├── launch/
        │   └── simulation.launch.py
        ├── config/
        │   ├── simulation.yaml
        │   └── simulation.rviz
        ├── pillar_camera_sim/
        │   ├── __init__.py
        │   ├── simulation_node.py
        │   ├── keyboard_teleop.py
        │   └── geometry.py
        ├── resource/
        └── test/
```

## 3. ビルド

ターミナル①:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select pillar_camera_sim
source install/setup.bash
ros2 pkg list | grep pillar_camera_sim
```

`pillar_camera_sim` と表示されればパッケージが認識されています。

> 新しいターミナルでは、ROS 2本体の `setup.bash` とワークスペースの `install/setup.bash` を再度読み込んでください。`$ROS_DISTRO` が未設定なら `source /opt/ros/humble/setup.bash` のように実際のディストリビューションを指定します。

## 4. シミュレーションとRViz2を起動

ターミナル①:

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch pillar_camera_sim simulation.launch.py
```

このlaunchは次の2つを起動します。

1. `simulation_node`: 柱のマーカー、仮想画像、TF、カメラ位置・推定値を配信
2. `rviz2`: `config/simulation.rviz` を読み込み、3Dシーンと画像を表示

RViz2の **Fixed Frame** は `map`、画像のトピックは `/camera/image_raw` です。

## 5. キーボードでカメラを動かす

**ターミナル②（新しいターミナル）**:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pillar_camera_sim keyboard_teleop
```

**操作はターミナル②にフォーカスして行います。** RViz2のウィンドウに直接キー入力しても移動しません。

| キー | カメラの動作 |
|---|---|
| `W` または `↑` | 前進：柱へ近づく（奥行き方向） |
| `S` または `↓` | 後退：柱から遠ざかる（奥行き方向） |
| `A` または `←` | 左に平行移動 |
| `D` または `→` | 右に平行移動 |
| `Q` | 左旋回 |
| `E` | 右旋回 |
| `R` | 上昇 |
| `F` | 下降 |
| `Space` | カメラを初期位置へ戻す |
| `X` または `Ctrl+C` | キーボード操作ノードを終了 |

初期設定では1キーにつき**移動0.10 m / 旋回5°**です。長押し時の連続入力はOSのキーリピート設定に依存します。柱は固定され、移動するのはカメラだけです。

## 6. 表示内容とトピック

| トピック / TF | 内容 |
|---|---|
| `/scene/markers` | 柱2本、地面、カメラ、視野線のRViz2マーカー |
| `/camera/image_raw` | 仮想カメラ画像と中心線 |
| `/camera/camera_info` | カメラ内部パラメータ |
| `/camera/pose` | カメラの位置・姿勢 |
| `/camera/estimate` | 画像上の中心ずれと推定距離 |
| `/camera/key_command` | キーボード操作コマンド（`std_msgs/msg/String`） |
| `map → camera_link → camera_optical_frame` | カメラ座標変換（TF） |

赤い縦線は**画像中心**、青い縦線は**2本の柱の中心**を示します。

柱の中心間隔を `B = 0.66 m`、画像上の柱中心間の画素差を `d_px`、焦点距離を `f_px` とすると、正面に近い場合の推定奥行きは次式です。

```text
推定距離 Z [m] = f_px × B / d_px
```

柱が画面から外れると推定できない場合があります。カメラが斜めを向く場合は単純な推定に誤差が出ます。

## 7. パラメータ変更

`config/simulation.yaml` を編集します。

| パラメータ | 初期値 | 意味 |
|---|---:|---|
| `pillar_distance_x` | `3.0` | 柱のX位置 [m] |
| `pillar_spacing` | `0.66` | 柱の中心間隔 [m] |
| `pillar_width` | `0.07` | 柱の幅 [m] |
| `pillar_height` | `1.65` | 柱の高さ [m] |
| `camera_start_x` | `0.0` | カメラ初期X位置 [m] |
| `camera_start_y` | `0.0` | カメラ初期Y位置 [m] |
| `camera_start_z` | `0.9` | カメラ初期高さ [m] |
| `camera_start_yaw` | `0.0` | カメラ初期旋回角 [rad] |
| `move_step` | `0.10` | 1入力の移動量 [m] |
| `yaw_step_deg` | `5.0` | 1入力の旋回量 [度] |
| `image_width` | `800` | 画像幅 [px] |
| `image_height` | `600` | 画像高さ [px] |
| `focal_length_px` | `700.0` | 焦点距離 [px] |

`--symlink-install` でビルドした場合も、設定の反映にはlaunchの再起動が必要です。通常のビルドでインストール済みファイルを更新する場合は再ビルドしてください。

## 8. 動作確認・デバッグ

ターミナル③:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list
ros2 topic list
ros2 topic echo /camera/pose
```

別のターミナルで推定値を確認:

```bash
ros2 topic echo /camera/estimate
```

キーボードノードを使わずに1回前進:

```bash
ros2 topic pub --once /camera/key_command std_msgs/msg/String "{data: 'w'}"
```

### RViz2だけを再起動する

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix pillar_camera_sim)/share/pillar_camera_sim/config/simulation.rviz"
```

このコマンドはRViz2だけを起動します。シミュレーションノードが動いていない場合、柱や画像は表示されません。

## 9. よくある問題

- **`Package 'pillar_camera_sim' not found`**: `package.xml` の配置を確認し、`colcon build --packages-select pillar_camera_sim` と `source install/setup.bash` を再実行してください。
- **`ros2 launch` でlaunchファイルが見つからない**: `launch/simulation.launch.py` が存在するか確認し、再ビルドしてください。
- **キーを押しても動かない**: `keyboard_teleop` を別の対話型ターミナルで起動し、そのターミナルをアクティブにしてください。
- **RViz2に何も表示されない**: Fixed Frameが `map` か、`/scene/markers` と `/camera/image_raw` が配信されているか確認してください。
- **`No module named cv2` / `numpy`**: `sudo apt install python3-opencv python3-numpy` を実行してください。
- **GUIが起動しない**: Ubuntuデスクトップ環境で実行してください。SSH環境ではX11/WaylandのGUI転送が別途必要です。

## 10. 終了方法

- ターミナル②: `X` または `Ctrl+C`
- ターミナル①: `Ctrl+C`（launchとRViz2を終了）

## 11. テスト

```bash
cd ~/ros2_ws
source install/setup.bash
colcon test --packages-select pillar_camera_sim
colcon test-result --verbose
```

