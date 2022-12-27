# xArm講習会
ここでは，本研究室の基本的なxArm6の動かし方について紹介する．
xArmの動かし方は基本的に二つある(他にもコマンド操作のようなものもあるがそれは難しいのでパス)．
1. UFactoryが出す[「UFACTORY Studio」](https://www.ufactory.cc/download-xarm-robot)というアプリケーションでマニュアル操作したり，ティーチングしたりする方法.
2. ROSとmoveitを駆使し，カメラなどのセンサから得た情報を基に軌道生成を行える開発者用の方法.

今回は，2のROSでxArmを動かす方法について紹介する．ROS2ではないのでROS2利用者は、ほんの参考程度に. できれば、ROS2バージョンも後輩たちの為に作ってもらえると助かります.

まず，大前提として，**ROSの環境構築(本研究室サーバー/manilab/software/ROS練習)** は終えておいてください．Teamsにも上がってるのでスマホで確認できます.
また，最低限のトピック通信の方法は学んでおいておくことを推奨します．

本資料の序盤部分は[公式のマニュアル](https://github.com/xArm-Developer/xarm_ros)に書いてあることをそのまま日本語化して、必要なものだけを載せただけです.もし何か分からないことあれば、最新の更新者に聞いてください.

※コマンドすべて頭文字に$が入っているので実行する前に取り除いてください.　
※目次が変ですが、日本語のタイトルにリンク貼れないようなのですいません...
**※所詮一学生が作ったものなので、正しい情報とは限りません. 分からない事は正直に分からないと書いてるので, その箇所が重要だと判断した際は、各自[公式のマニュアル](https://github.com/xArm-Developer/xarm_ros)から理解しこの資料の更新をお願いします.** 

# 本資料の更新日及び更新者
後輩たちがxArmを使った研究を行う際に、「アームの使い方分からない！」とならないようにする為に作成した資料です. 
もし、xarm等のパッケージに更新があり、本資料ではエラーが出てしまう場合は、お手数ですが更新をお願いします.github分からないかもしれないですけど、なんとなくこの資料のRawデータ見ればわかるはずですので頑張ってください. また、最初の編集者(荒木)は広く浅くなので、所々解説が雑です. 深い部分まで理解が出来た方はドンドン書き加えて、間違ってたら直してください！
|更新日|氏名|配属|mail(任意)|
| ------------- | ------------- |------------- | ------------- |
| 2020/12/27| 荒木 博揚 |2020(15期生)| hiroark.mani@gmail.com |


# 目次:  
* [1. 依存パッケージのインストール (**必須**)](#1-preparations-before-using-this-package)
* [2. 環境構築](#2-getting-started-with-xarm_ros)
* [3.  パッケージの説明と使用上の注意](#3-package-description--usage-guidance)
    * [3.1 xarm_description](#31-xarm_description)  
    * [3.2 xarm_gazebo](#32-xarm_gazebo)  
    * [3.3 xarm_controller](#33-xarm_controller)  
    * [3.4 xarm_bringup](#34-xarm_bringup)  
    * [3.5 ***xarm6_moveit_config***](#35-xarm6_moveit_config)  
    * [3.6 ***xarm_planner***](#36-xarm_planner)  
    * [3.7 ***xarm_api/xarm_msgs (Online Planning Modes Added)***](#37-xarm_apixarm_msgs)  
* [6. Mode Change(***Updated***)](#6-mode-change)
    * [6.1 Mode Explanation](#61-mode-explanation)
    * [6.2 Proper way to change modes](#62-proper-way-to-change-modes)
* [7. xArm Vision](#7-xarm-vision)
    * [7.1 Installation of dependent packages](#71-installation-of-dependent-packages)
    * [7.2 Hand-eye Calibration Demo](#72-hand-eye-calibration-demo)
    * [7.3 Vision Guided Grasping Demo](#73-vision-guided-grasping-demo)
    * [7.4 Adding RealSense D435i model to simulated xArm](#74-adding-realsense-d435i-model-to-simulated-xarm)
    * [7.5 Color Cube Grasping Demo (Simulation + Real Hardware)](#75-color-cube-grasping-demo)
* [8. Other Examples](#8-other-examples)
    * [8.0 An example of demonstrating redundancy resolution using MoveIt](https://github.com/xArm-Developer/xarm_ros/tree/master/examples/xarm7_redundancy_res)
    * [8.1 Multi-xArm5 (separate control)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#1-multi_xarm5-controlled-separately)
    * [8.2 Servo_Cartesian](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)
    * [8.3 Servo_Joint](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory)
    * [8.4 Dual xArm6 controlled with one moveGroup node](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#4-dual-xarm6-controlled-with-one-movegroup-node)
    * [8.5 Record and playback trajectories](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#5-run-recorded-trajectory-beta)
    * [8.6 Online target update for dynamic following task(**NEW**)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update)


# 1. Preparations before using this package

## 1.1 依存パッケージのインストール
まず、これらのパッケージをインストールしてください. 

   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> (ガゼボ(シミュレーション)使う人は必須)   
   ros_control: <http://wiki.ros.org/ros_control> (ROSのバージョンに注意)  
   moveit_core: <https://moveit.ros.org/install/>  
   
## 1.2 これらパッケージの参考
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

# 2. Getting started with xarm_ros

## 2.1 ワークスペースの作成. 
   &ensp;&ensp;catkin_wsはROSの環境構築で作ったはずなのでここは飛ばしてください.
   もし違うワークスペースでやりたいという人は[このページ](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)を参考に新たにワークスペースを作成してください. 
   この資料内でのディレクトリ移動等のコマンドはすべて"catkin_ws"を使っているので、新たに作った人は各自書き換えてください.

## 2.2 パッケージの取得
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git --recursive
   ```

## 2.2.1　パッケージのアップデート
   ```bash
   $ cd ~/catkin_ws/src/xarm_ros
   $ git pull
   $ git submodule sync
   $ git submodule update --init --remote
   ```

## 2.3 依存パッケージのアップデート:
   ```bash
   $ rosdep update
   $ rosdep check --from-paths . --ignore-src --rosdistro noetic
   ```
   ROSのバージョンがnoeticじゃ無い場合は、各自変えること.

## 2.4 ビルド
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 2.5 バッシュ
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
すでに~/.bashrcの中にある場合は、上記の操作をスキップしてください. (.bashrcの中身を確認してください)
次に、この操作を行います。:
```bash
$ source ~/.bashrc
```
## 2.6 テスト動作:
```bash
$ roslaunch xarm_description xarm6_rviz_display.launch
```
こんな表示になれば成功！もしかしたら、研究室の机のモデル等が入ったりするかもだけど、それでもこんなRviz画面が表示されれば取り敢えずok!
![Screenshot from 2022-12-27 19-40-04](https://user-images.githubusercontent.com/86779771/209654693-896efd9e-a1f9-4abd-9666-d9073d3bc46d.png)

上手く行かない場合は、エラー文をググって見てください. それでもわからない場合は最初からやり直しましょう.

## 2.7 gazeboでシミュレーション
   ```bash
   $ roslaunch xarm_gazebo xarm6_beside_table.launch [run_demo:=true] [add_gripper:=true] [add_vacuum_gripper:=true] 
   ```
こんな表示になれば成功
![Screenshot from 2022-12-27 20-03-22](https://user-images.githubusercontent.com/86779771/209657584-aa987b5b-ff25-4aa6-9b32-dce5e9c54886.png)

端末の方で、
```bash
[ERROR] [1672138940.247346024, 0.356000000]: No p gain specified for pid.  Namespace: /xarm/gazebo_ros_control/pid_gains/joint1
[ERROR] [1672138940.248592258, 0.356000000]: No p gain specified for pid.  Namespace: /xarm/gazebo_ros_control/pid_gains/joint2
[ERROR] [1672138940.249474143, 0.356000000]: No p gain specified for pid.  Namespace: /xarm/gazebo_ros_control/pid_gains/joint3
[ERROR] [1672138940.250319620, 0.356000000]: No p gain specified for pid.  Namespace: /xarm/gazebo_ros_control/pid_gains/joint4
[ERROR] [1672138940.251141814, 0.356000000]: No p gain specified for pid.  Namespace: /xarm/gazebo_ros_control/pid_gains/joint5
[ERROR] [1672138940.251953284, 0.356000000]: No p gain specified for pid.  Namespace: /xarm/gazebo_ros_control/pid_gains/joint6
```
こういう赤字のエラーが出ると思いますが、これは無視して結構です. (上手く消せたら更新お願いします〜　by荒木)

他にも、黄色の文字でWarnig!の警告文も出ますが、出るものです！笑　(同様に消せたら更新を！)

# 3. Package description & Usage Guidance
   
## 3.1 xarm_description
   &ensp;&ensp;xArmの記述ファイル. メッシュファイル、gazeboのプラグイン設定など. xarmの記述ファイルは、他のパッケージが依存しているので、変更することはお勧めしない.
   URDFファイルなど編集する場合(研究室の環境情報(机等)を書きたい場合)はここを弄ります. 編集の方法などは後述.

## 3.2 xarm_gazebo
   &ensp;&ensp;Gazeboワールド記述ファイル、シミュレーション起動ファイル. シミュレーションワールドファイルには、ユーザが独自にモデルを追加・構築することができる. 
   らしいんですけど、実機で試行錯誤していた荒木には詳しいことはわかりません.(Gazeboで頑張る方は更新お願いします)

## 3.3 xarm_controller
   &ensp;&ensp;コントローラ設定、Hardware_Interface、ロボットコマンド実行可能なソース、スクリプト、起動ファイル。ユーザはこのパッケージの中に自分のプログラムを配置することもできますし、自分で作成することもできます。***注意：*** xarm_controller/config で定義されているコントローラはシミュレーションのための例であり、実機の制御には 'position_controllers/JointTrajectoryController' インターフェースのみが提供されています。ユーザが自分で定義したコントローラを追加することもできます。http://wiki.ros.org/ros_control (controllers) を参照してください.
   
   と色々説明してますけど、荒木の研究では一切ここは弄ってないので正直良くわかりません笑
   
## 3.4 xarm_bringup  
&ensp;&ensp;xarmドライバをロードし、実際のxArmハードウェアを直接制御できるようにするための起動ファイルです. 荒木の研究では特に弄っていませんが、実機を起動する際に重要なファイルです. 起動の仕方などは後述します.

## 3.5 xarm6_moveit_config
xarm_moveit_config 関連パッケージは、すべての関節を [-pi, pi] の範囲に制限します。この範囲に制限しない場合、moveit はより大きな関節運動を含む計画を生成する傾向があるようです。この制限は、...moveit_config/launch/planning_context.launch で limited:=false とすることにより解除することができる. 

↑に関しては試したことがないのでよくわかりません.

&ensp;&ensp;本パッケージは、moveit_setup_assistantによって部分的に生成され、Moveit PlannerやRvizによる可視化で使用することができる.Moveit! がインストールされていれば、デモを試すことができる.
   ```bash
   $ roslaunch xarm6_moveit_config demo.launch
   ```
   こんな表示になれば成功！
   ![Screenshot from 2022-12-27 21-02-30](https://user-images.githubusercontent.com/86779771/209664164-ba5168d4-1114-490a-bed8-61a9c67f6644.png)
   
   こんな感じに手動で動かせるか確認をしよう.
   
   [Screencast from 2022年12月27日 21時02分35秒.webm](https://user-images.githubusercontent.com/86779771/209665020-38b6d7f1-271a-4ec2-8217-6a4891fb2769.webm)
#### Gazeboシミュレータと一緒にMoveit！モーションプランナーを実行:  
   1. **xArmグリッパーが必要ない場合**は、まずこれを実行:  
   ```bash
   $ roslaunch xarm_gazebo xarm6_beside_table.launch
   ```
   次に、別のターミナルで:
   ```bash
   $ roslaunch xarm6_moveit_config xarm6_moveit_gazebo.launch
   ```
   2. **xArmグリッパーが必要な場合**:  
   ```bash
   $ roslaunch xarm_gazebo xarm6_beside_table.launch add_gripper:=true
   ```
   次に、別のターミナルで:
   ```bash
   $ roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch
   ```
   Moveit！で満足のいく動作が計画できたら、「Excute」ボタンを押すと、Gazebo の仮想アームが軌道を実行する.

   3.  **xArmのバキュームグリッパーが必要な場合**, 上記のグリッパーの例の "gripper "を "vacuum_gripper "に置き換えるだけでよいでしょう。

#### Moveit！モーションプランナーで実際のxArm(実機)を制御する方法:  
   まず、xArmとコントローラボックスの電源が入っていること(**緊急停止ボタンをOFFにし電源ボタンがON**)を確認し、以下のコマンドを実行:  
   ```bash
   $ roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.217 
   ```
   正しく動作した際は、xArmが「カチカチカチ」と内部アクチュエータが起動した音が鳴ります. (元から起動している場合は鳴りません.)ターミナルの出力を見て、起動中にエラーが発生したかどうかを確認します. ターミナルの出力を見て、起動中に何かエラーが発生したかどうか確認してください。もしエラーがなければ、Rvizで先の動画のようにロボットを簡単に操作し、うまく計画された軌道を実際のアームで実行することができます。ただし、実行前に周囲にぶつからないことを確認してください!　

※robot_ipはアーム毎に異なります. もし二台目のxArmなどが入荷された場合は、ここのIPはそれに変えるようにしてください.

#### xArm Gripperを実際に入荷し取り付けた場合:  
   ```bash
   $ roslaunch xarm6_gripper_moveit_config realMove_exec.launch robot_ip:=192.168.1.217 
   ```
     Moveitプランナーはグリッパーを考慮して衝突判定を行うので、このパッケージは実際のxArmグリッパーと一緒に使うのがよいでしょう。 

#### xArm Vaccum Gripperを実際に入荷し取り付けた場合:  
   ```bash
   $ roslaunch xarm7_vacuum_gripper_moveit_config realMove_exec.launch robot_ip:=192.168.1.217
   ```

## 3.5.1 Moveitのためのカスタムツールモデルの追加
&ensp;&ensp;xarm5_moveit_config__/__xarm6_moveit_config__/__xarm7_moveit_config__ では、以下のクイック設定パラメータにより、カスタマイズした工具モデルを工具フランジに追加し、 Moveit動作計画時の工具オフセットと3次元干渉チェックを行うことができます。(注意：'/xarm/set_tcp_offset' サービスによる設定は、Moveit の動作計画には有効ではありません！) ### 例：'/xarm/set_tcp_offset' サービスによる設定は、動作計画には有効ではありません。

### Examples:
   ```bash
   # 直方体のモデルの取り付け:
   $ roslaunch xarm6_moveit_config demo.launch add_other_geometry:=true geometry_type:=box

   # 円筒形状のモデル:
   $ roslaunch xarm6_moveit_config demo.launch add_other_geometry:=true geometry_type:=cylinder

   # 球モデル:
   $ roslaunch xarm6_moveit_config demo.launch add_other_geometry:=true geometry_type:=sphere

   # カスタマイズモデル(設計したやつとか):（ここでは、xarm vacuum_gripperを例とします。xarm_description/mmeshes/other'ディレクトリに置く場合、geometry_mesh_filenameは、ファイル名として簡略化できます）
   $ roslaunch xarm6_moveit_config demo.launch add_other_geometry:=true geometry_type:=mesh geometry_mesh_filename:=package://xarm_description/meshes/vacuum_gripper/visual/vacuum_gripper.STL geometry_mesh_tcp_xyz:='"0 0 0.126"'
   ```
   このような形で設計したものをSTL形式にすれば、それをPCのシミュレーション上に投影し動作計画時に干渉チェックできる.試しに机の上にbox置いてみるとかでは使えるけど、ハンドを取り付ける場合毎回これやるのはバカバカしいのでオススメしません.(後述)

### 引数の説明:
- __add_other_geometry__:デフォルトはfalse，他のジオメトリモデルをツールに追加するかどうかを示す．
- __geometry_type__: box/cylinder/sphere/meshのいずれかを選択し、種類によって必要なパラメータが異なる。 
- __geometry_height__: ジオメトリの高さ，単位: m，デフォルト値: 0.1，ジオメトリタイプ: box/cylinder/sphere に適用されます。
- __geometry_radius__: ジオメトリの半径，単位: メートル，デフォルト値: 0.1，ジオメトリの種類: 円柱/球に対して有効
- __geometry_length__: ジオメトリの長さ，単位: メートル，デフォルト値: 0.1，ジオメトリの種類: 箱に適用されます。
- __geometry_width__: ジオメトリの幅，単位: m，デフォルト値: 0.1，ジオメトリの種類: box に対して有効。
- __geometry_mesh_filename__: ジオメトリ形状、ジオメトリタイプ：メッシュに対して有効。
- __geometry_mesh_origin_xyz__:メッシュのベース座標から xarm ツールフランジ座標へのオフセット、デフォルト: "0 0 0"、ジオメトリの種類: メッシュに効果的です。
- __geometry_mesh_origin_rpy__: メッシュのベース座標から xarm ツールフランジ座標への方向オフセット、デフォルト: "0 0 0"、ジオメトリの種類: メッシュに有効です。
- __geometry_mesh_tcp_xyz__: xarm tool-flange 座標に対する TCP オフセット、デフォルト: "0 0 0"、geometry_type: mesh に対して有効です。
- __geometry_mesh_tcp_rpy__:xarm ツールフランジ座標に対する方向 TCP オフセット、デフォルト: "0 0 0"、ジオメトリタイプ: メッシュに対して有効。 

## 3.6 xarm_planner:
&ensp;&ensp;この実装は，Moveit!のmove_groupをベースとした，シンプルなプランナインタフェースで，ユーザが要求したターゲットに応じた計画・実行を行うためのサービスを提供します．
※ここも特に弄ってないのでよく分かりません.

## 3.7 xarm_api/xarm_msgs:
&ensp;&ensp;この2つのパッケージは、xArm SDKの機能のROSサービスラッパーをユーザーに提供します。12種類のモーションコマンド(サービス名)がサポートされていますが、まず正しいロボットモードを設定してください。 

ここも荒木は弄ってないのでよく分かりません. 

#### ROSのサービスからも動かせる！:
Rviz&moveitではなく、コマンドからも動かすことが可能です.

&ensp;&ensp;まず、xarm6 のサービスサーバーを起動します:  
```bash
$ roslaunch xarm_bringup xarm6_server.launch robot_ip:=192.168.1.217 report_type:=normal
```

&ensp;&ensp;次に、すべてのサーボモーターが有効になっていることを確認します:
```bash
$ rosservice call /xarm/motion_ctrl 7 1
```
&ensp;&ensp;モーションコマンドを実行する前に、ロボットモード（0：POSE）と状態（0：READY）を 設定  :    
```bash
$ rosservice call /xarm/clear_err
$ rosservice call /xarm/set_mode 0

$ rosservice call /xarm/set_state 0
```
これで、xArmが「カチカチカチ」となるはずです.(この辺は使ってないので自信ないです)

#### 位置制御の例:
&ensp;&ensp;角度は全て ***radian*** で指定することに注意してください.

##### 1. 関節空間運動:
&ensp;&ensp;最大速度 0.35 rad/s および加速度 7 rad/s^2 でジョイント スペース モーションを呼び出すには::   
```bash
$ rosservice call /xarm/move_joint [0,0,0,0,0,0,0] 0.35 7 0 0
```
&ensp;&ensp;最大速度 0.35 rad/s、加速度 7 rad/s^2 でホーム (すべてのジョイントが 0 rad) 位置に戻るには::  
```bash
$ rosservice call /xarm/go_home [] 0.35 7 0 0
```
##### 2. ベース座標でのデカルト空間の動き:
&ensp;&ensp;最大速度 200 mm/s および加速度 2000 mm/s^2 で、ロボットの BASE 座標で表されたターゲットに直交運動を呼び出すには:
```bash
$ rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
```
##### 3. ツール座標でのデカルト空間の動き:
&ensp;&ensp; ロボット ツール座標で表現された直交運動を呼び出すには、最大速度 200 mm/s および加速度 2000 mm/s^2 で、以下は現在のツールに沿って相対運動(delta_x=50mm、delta_y=100mm、delta_z=100mm)を移動します。座標、方向変更なし:
$ rosservice call /xarm/move_line_tool [50,100,100,0,0,0] 200 2000 0 0
```
##### 4.  軸角方向のデカルト空間運動:
&ensp;&ensp;  軸角度モーションに対応するサービスはMoveAxisAngle.srvです。最後の 2 つの引数に注意してください。「coord」は、(wrt) アーム ベース座標系に対するモーションの場合は 0、ツール座標系に対するモーションの場合は 1 です。" relative " は、指定された座標系に対する絶対ターゲット位置の場合は 0、相対ターゲット位置の場合は 1 です。
  例: ツール フレームの Z 軸を中心に相対的に 1.0 ラジアン移動するには: 
```bash
$ rosservice call /xarm/move_line_aa "pose: [0, 0, 0, 0, 0, 1.0]
mvvelo: 30.0
mvacc: 100.0
mvtime: 0.0
coord: 1
relative: 1" 
ret: 0
message: "move_line_aa, ret = 0"
```
または
```bash
$ rosservice call /xarm/move_line_aa [0,0,0,0,0,1.0] 30.0 100.0 0.0 1 1
```   
&ensp;&ensp; " mvtime " はこのコマンドでは意味がありません。単に 0 に設定してください。別の例: base-frame で、Y 軸に沿って 122mm 相対的に移動し、X 軸を中心に -0.5 ラジアン回転します。
$ rosservice call /xarm/move_line_aa [0,122,0,-0.5,0,0] 30.0 100.0 0.0 0 1  
```


####ステータス フィードバックの取得:
&ensp;&ensp;  「xarm7_server.launch」を実行して実際の xArm ロボットに接続すると、ユーザーはトピック「xarm/xarm_states」にサブスクライブして、関節角度、TCP 位置、エラー/警告コードなどを含む現在のロボットの状態に関するフィードバック情報を得ることができます。コンテンツの詳細については、 RobotMsg.msgを参照してください。
  別のオプションは、「/joint_states」トピックをサブスクライブすることです。これはJointState.msgでレポートされますが、現在は「位置」フィールドのみが有効です; 「速度」は、隣接する 2 つの位置データに基づくフィルタリングされていない数値微分であり、「努力」フィードバックは、直接トルク センサーからではなく、電流ベースの推定値であるため、参考用です。パフォーマンスを考慮して、上記 2 つのトピックのデフォルトの更新レートは5Hzに設定されています。レポートの内容と頻度には他のオプションがあります。report_type 引数を参照してください。![xArmFrames](https://user-images.githubusercontent.com/86779771/209673725-c1877d63-0cdd-4a2d-9bad-206d95fc360b.png)


#### Setting Tool Center Point Offset(only effective for xarm_api ROS service control):
&ensp;&ensp;ツール チップ ポイントのオフセット値は、サービス「/xarm/set_tcp_offset」を呼び出すことで設定できます。下の図を参照してください。このオフセット座標は、ベース フレームから (PI, 0, 0) のロール、ピッチ、ヨー回転で、フランジの中心に位置する既定のツール フレーム(フレーム B) に対して表されていることに注意してください (フレーム A)。   例えば：
![Uploading xArmFrames.png…]()  
```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp; これは、ツール フレームの位置オフセット (x = 0 mm、y = 0 mm、z = 20 mm) と、最初のツール フレーム (フレーム B のフレーム B写真）。このオフセットは、スタジオで設定されたデフォルト値と一致しない場合、xArm Stdudio によって上書きされる可能性があることに注意してください。ros サービス コントロールと一緒に使用する場合は、xArm studio で同じ TCP デフォルト オフセット設定を行うことをお勧めします。  

#### エラーのクリア:
&ensp;&ensp;コントローラーは、その後のコマンドの実行に影響を与えるエラーまたは警告を報告する場合があります。理由としては、電力損失、位置/速度制限違反、計画エラーなどが考えられます。クリアするには追加の介入が必要です。ユーザーは、トピック"xarm/xarm_states"のメッセージでエラー コードを確認できます。
```bash
$ rostopic echo /xarm/xarm_states
```
&ensp;&ensp;ゼロ以外の場合、対応する理由はユーザーマニュアルで見つけることができます。問題を解決した後、空の引数でサービス"/xarm/clear_err"を呼び出すことにより、このエラー状態を取り除くことができます。
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;  Moveit! を使用している場合は、代わりに " /xarm/moveit_clear_err " を呼び出して、手動でモード 1 を再度設定する必要がないようにしてください。
```bash
$ rosservice call /xarm/moveit_clear_err
```
&ensp;&ensp;  このサービスを呼び出した後、「xarm/xarm_states」でエラー ステータスをもう一度確認してください。0 になっていれば、クリアは成功です。それ以外の場合は、エラー/例外が適切に解決されていないことを意味します。エラーのクリアに成功した場合は、ロボットの状態を 0に設定して、再び移動できるようにすることを忘れないでください。

# 6. Mode Change
&ensp;&ensp;xArm may operate under different modes depending on different controling methods. Current mode can be checked in the message of topic "xarm/xarm_states". And there are circumstances that demand user to switch between operation modes. 

### 6.1 Mode Explanation

&ensp;&ensp; ***Mode 0*** : xArm controller (Position) mode.  
&ensp;&ensp; ***Mode 1*** : External trajectory planner (position) mode.  
&ensp;&ensp; ***Mode 2*** : Free-Drive (zero gravity) mode.  
&ensp;&ensp; ***Mode 3*** : Reserved.  
&ensp;&ensp; ***Mode 4*** : Joint velocity control mode.  
&ensp;&ensp; ***Mode 5*** : Cartesian velocity control mode.  
&ensp;&ensp; ***Mode 6*** : Joint space online planning mode. (Firmware >= v1.10.0)  
&ensp;&ensp; ***Mode 7*** : Cartesian space online planning mode. (Firmware >= v1.11.0)  

&ensp;&ensp;***Mode 0*** is the default when system initiates, and when error occurs(collision, overload, overspeed, etc), system will automatically switch to Mode 0. Also, all the motion plan services in [xarm_api](./xarm_api/) package or the [SDK](https://github.com/xArm-Developer/xArm-Python-SDK) motion functions demand xArm to operate in Mode 0. ***Mode 1*** is for external trajectory planner like Moveit! to bypass the integrated xArm planner to control the robot. ***Mode 2*** is to enable free-drive operation, robot will enter Gravity compensated mode, however, be sure the mounting direction and payload are properly configured before setting to mode 2. ***Mode 4*** is to control arm velocity in joint space. ***Mode 5*** is to control arm (linear) velocity in Cartesian space. ***Mode 6 and 7*** are for dynamic realtime response to newly generated joint or Cartesian target respectively, with automatic speed-continuous trajectoty re-planning.

### 6.2 Proper way to change modes:  
&ensp;&ensp;If collision or other error happens during the execution of a Moveit! planned trajectory, Mode will automatically switch from 1 to default mode 0 for safety purpose, and robot state will change to 4 (error state). The robot will not be able to execute any Moveit command again unless the mode is set back to 1. The following are the steps to switch back and enable Moveit control again:  

&ensp;&ensp;(1) Make sure the objects causing the collision are removed.  
&ensp;&ensp;(2) clear the error:  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;(3) switch to the desired mode (Mode 2 for example), and set state to 0 for ready:
```bash
$ rosservice call /xarm/set_mode 2

$ rosservice call /xarm/set_state 0
```

# 7. xArm Vision
For simple demonstrations of vision application development with xArm, including hand-eye calibration and object detection and grasping. Examples are based on [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) depth camera.

## 7.1 Installation of dependent packages:

First enter the workspace source directory:
```bash
$ cd ~/catkin_ws/src/
```

### 7.1.1 Install RealSense developer library and ROS package： 
Please refer to the installation steps at [official webpage](https://github.com/IntelRealSense/realsense-ros).

### 7.1.2 Install 'aruco_ros', for hand-eye calibration：
Refer to [official Github](https://github.com/pal-robotics/aruco_ros):
```bash
$ git clone -b kinetic-devel https://github.com/pal-robotics/aruco_ros.git
```
### 7.1.3 Install 'easy_handeye', for hand-eye calibration：
Refer to [official Github](https://github.com/IFL-CAMP/easy_handeye):
```bash
$ git clone https://github.com/IFL-CAMP/easy_handeye
``` 
### 7.1.4 Install 'vision_visp' supporting package：
Refer to [official Github](https://github.com/lagadic/vision_visp):
```bash
$ git clone -b kinetic-devel https://github.com/lagadic/vision_visp.git
```
### 7.1.5 Install 'find_object_2d', for object detection：
Refer to [official Github](https://github.com/introlab/find-object/tree/kinetic-devel):
```bash
$ sudo apt-get install ros-kinetic-find-object-2d
```
### 7.1.6 Install other dependencies：
```bash
$ cd ~/catkin_ws
```
Then follow chapter [4.3](#43-install-other-dependent-packages).

### 7.1.7 Build the whole workspace：
```bash
$ catkin_make
```

## 7.2 Hand-eye Calibration Demo：
If attaching RealSense D435i camera at tool end of xArm, with mechanical adapters, making a "**eye-on-hand**"(or eye-in-hand) configuration，the following launch file can be used and modified for hand-eye calibration: (make sure the camera communication is functional and robot is properly switched on)
```bash
$ roslaunch d435i_xarm_setup d435i_xarm_auto_calib.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
The `aruco Marker` used inside can be downloaded [here](https://chev.me/arucogen/), please remember the `marker ID` and `marker size` and modify them in the launch file accordingly. Refer to [official](https://github.com/IFL-CAMP/easy_handeye#calibration)or other usage instructions online and finish the calibration with the GUI.   

If calculation result is confirmed and saved，it will appear by default under `~/.ros/easy_handeye` directory and can be used for transferring object coordinates to base frame. If the [camera_stand](https://www.ufactory.cc/products/xarm-camera-module-2020) provided by UFACTORY is used for fixing camera, a sample calibration result is stored at xarm_vision/d435i_xarm_setup/config/[xarm_realsense_handeyecalibration_eye_on_hand_sample_result.yaml](./xarm_vision/d435i_xarm_setup/config/xarm_realsense_handeyecalibration_eye_on_hand_sample_result.yaml) for this case.  

## 7.3 Vision Guided Grasping Demo:
[***find_object_2d***](http://introlab.github.io/find-object/) is used for this demo for simple object detection and grasping. Hardware used in this part: RealSense D435i depth camera, UFACTORY camera stand and the xArm Gripper.  

1.Use moveit to drive xArm's motion，recommended for singularity and collision free execution, but will require a reliable network connection.  
```bash
$ roslaunch d435i_xarm_setup d435i_findobj2d_xarm_moveit_planner.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
If target object can be properly detected, to run the Grasping node:  
```bash
$ rosrun d435i_xarm_setup findobj2d_grasp_moveit
```

Please note it will use previously mentioned sample handeye calibration result, you can change it at [publish_handeye_tf.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf.launch). For node program source code, refer to: d435i_xarm_setup/src/[findobj_grasp_moveit_planner.cpp](./xarm_vision/d435i_xarm_setup/src/findobj_grasp_moveit_planner.cpp).  

2.Alternatively, to drive xArm motion with ros service provided by 'xarm_api', in this way, real-time performance of network will not be required so strict as moveit way, but execution may fail in the middle if singularity or self-collision is about to occur. 
```bash
$ roslaunch d435i_xarm_setup d435i_findobj2d_xarm_api.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
If target object can be properly detected, to run the Grasping node:  
```bash
$ roslaunch d435i_xarm_setup grasp_node_xarm_api.launch
```
Please note it will use previously mentioned sample handeye calibration result, you can change it at [publish_handeye_tf.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf.launch). For node program source code, refer to: d435i_xarm_setup/src/[findobj_grasp_xarm_api.cpp](./xarm_vision/d435i_xarm_setup/src/findobj_grasp_moveit_xarm_api.cpp).  

***Please read and comprehend the source code and make necessary modifications before real application test***, necessary modifications include preparation pose, grasping orientation, grasping depth, motion speed and so on. The identification target name in the code is "object_1", which corresponds to `1.png` in /objects directory, users can add their own target in "find_object_2d" GUI, then modify the `source_frame` inside the code, for costomized application.  

***Tips***: make sure the background is clean and the color is distinguished from the object, detection success rate can be higher if the target object has rich texture (features).

## 7.4 Adding RealSense D435i model to simulated xArm：
For installation with camera stand provided by UFACTORY, the cam model can be attached by following modifications (use xarm7 as example):    
1.Together with xArm Gripper model: Set `add_realsense_d435i` default value to be `true` in [xarm7_with_gripper.xacro](./xarm_description/urdf/xarm7_with_gripper.xacro).  
2.Together with xArm Vacuum Gripper model: Set `add_realsense_d435i` default value to be `true` in [xarm7_with_vacuum_gripper.xacro](./xarm_description/urdf/xarm7_with_vacuum_gripper.xacro).  
3.Purely the d435i: Set `add_realsense_d435i` default value to be `true` in [xarm7_robot.urdf.xacro](./xarm_description/urdf/xarm7_robot.urdf.xacro).  

## 7.5 Color Cube Grasping Demo

### 7.5.1 Download 'gazebo_grasp_plugin' for successful grasp simulation (ROS Melodic and later)
```bash
 # enter source directory of ROS workspace:
 $ cd ~/catkin_ws/src/
 # Download through git (mind to checkout the proper branch):
 $ git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
 # Compile:
 $ cd ..
 $ catkin_make
```
### 7.5.2 Gazebo grasping simulation (ROS Melodic and later)
```bash
 # Initialize gazebo scene and move_group:
 $ roslaunch xarm_gazebo xarm_camera_scene.launch robot_dof:=6

 # In another terminal, run the color recognition and grasping script:
 $ rosrun xarm_gazebo color_recognition.py
```
### 7.5.3 Real xArm and Intel realsense_d435i hardware
```bash
 # launch move_group:
 $ roslaunch camera_demo xarm_move_group.launch robot_ip:=192.168.1.15 robot_dof:=6

 # In another terminal, run the color recognition and grasping script (use with interaction prompt):
 $ rosrun camera_demo color_recognition.py
```


# 8. Other Examples
&ensp;&ensp;There are some other application demo examples in the [example package](./examples), which will be updated in the future, feel free to explore it.
