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

**※所詮一学生が作ったものなので、正しい情報とは限りません. 分からない事は正直に分からないと書いてるので, その箇所が重要だと判断した際は、各自[公式のマニュアル](https://github.com/xArm-Developer/xarm_ros)から理解しこの資料の更新をお願いします.** 

# 本資料の更新日及び更新者
後輩たちがxArmを使った研究を行う際に、「アームの使い方分からない！」とならないようにする為に作成した資料です. 
もし、xarm等のパッケージに更新があり、本資料ではエラーが出てしまう場合は、お手数ですが更新をお願いします.github分からないかもしれないですけど、なんとなくこの資料のRawデータ見ればわかるはずですので頑張ってください. また、最初の編集者(荒木)は広く浅くなので、所々解説が雑です. 深い部分まで理解が出来た方はドンドン書き加えて、間違ってたら直してください！
|更新日|氏名|配属|mail(任意)|
| ------------- | ------------- |------------- | ------------- |
| 2022/12/28| 荒木 博揚 |2020(15期生)| hiroark.mani@gmail.com |


# 目次:  
* [1. 依存パッケージのインストール (**必須**)](#1-preparations-before-using-this-package)
* [2. 環境構築](#2-getting-started-with-xarm_ros)
* [3.  パッケージの説明と使用上の注意](#3-package-description--usage-guidance)
    * [3.1 xarm_description](#31-xarm_description)  
    * [3.2 xarm_gazebo](#32-xarm_gazebo)  
    * [3.3 xarm_controller](#33-xarm_controller)  
    * [3.4 xarm_bringup](#34-xarm_bringup)  
    * [3.5 xarm6_moveit_config(**重要**)](#35-xarm6_moveit_config)
    ←ぶっちゃけココ出来てしまえばいいよ！！他は知っておいたらいいかも程度.  
    * [3.6 xarm_planner](#36-xarm_planner)  
    * [3.7 xarm_api/xarm_msgs (Online Planning Modes Added)](#37-xarm_apixarm_msgs)  
* [4. 制御モードの変更](#4-mode-change)
    * [4.1 Mode Explanation](#41-mode-explanation)
    * [4.2 Proper way to change modes](#42-proper-way-to-change-modes)
* [5.Pythonで動かす方法](#5-python-control)

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
また、毎回
   ```bash
   $ cd ~/catkin_ws
   ```
   でワークスペースに入ってからlaunchファイル等の実行を行ってください.

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
   1. **エンドエフェクタがない場合**:  
   ```bash
   $ roslaunch xarm_gazebo xarm6_beside_table.launch
   ```
   次に、別のターミナルで:
   ```bash
   $ roslaunch xarm6_moveit_config xarm6_moveit_gazebo.launch
   ```
   先の動画のように動かしてみると、Gazeboシミュレータ上のロボットが動作することが確認できます.
   ![Screenshot from 2022-12-28 16-10-33](https://user-images.githubusercontent.com/86779771/209773114-322ec31c-7d47-4550-9891-0ed055abd84a.png)
   

   2. **xArmグリッパー(UFactory製品)が必要な場合(任意)**:  
   ```bash
   $ roslaunch xarm_gazebo xarm6_beside_table.launch add_gripper:=true
   ```
   次に、別のターミナルで:
   ```bash
   $ roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch
   ```
   Moveit！で満足のいく動作が計画できたら、「Excute」ボタンを押すと、Gazebo の仮想アームが軌道を実行する.

   3.  **xArmのバキュームグリッパー(UFactory製品)が必要な場合(任意)**:


   上記のグリッパーの例の "gripper "を "vacuum_gripper "に置き換えるだけでよいでしょう。

#### Moveit！モーションプランナーで実際のxArm(実機)を制御する方法(重要):  
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
   ![Screenshot from 2022-12-28 17-43-22](https://user-images.githubusercontent.com/86779771/209784417-3cb955a6-30a2-4b73-b051-1868ade53cbe.png)
   
   このような形で設計したものをSTL形式にすれば、それをPCのシミュレーション上に投影し、エンドエフェクタとして装着でき動作計画時に干渉チェックできる.　ただ、コマンドでいちいち呼び出さなければいけないので、ハンドを取り付ける場合はあまりこの方法は推奨できない(後述).

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

#### ROSのサービスからも動かせます(任意、たまに使います):
Rviz&moveitではなく、コマンドからも動かすことが可能です.

&ensp;&ensp;まず、xarm6 のサービスサーバーを起動します:  
```bash
$ roslaunch xarm_bringup xarm6_server.launch robot_ip:=192.168.1.217 report_type:=normal
```

&ensp;&ensp;次に、すべてのサーボモーターが有効になっていることを確認します:
```bash
$ rosservice call /xarm/motion_ctrl 7 1
```

ここで、xArmが「カチカチカチ」となり以下の表示が出ます.

```bash
ret: 0
message: "motion enable, ret = 0"
```

&ensp;&ensp;モーションコマンドを実行する前に、ロボットモード（0：POSE）と状態（0：READY）を 設定  :    
```bash
$ rosservice call /xarm/clear_err
$ rosservice call /xarm/set_mode 0
$ rosservice call /xarm/set_state 0
```

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
```bash
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
```bash
$ rosservice call /xarm/move_line_aa [0,122,0,-0.5,0,0] 30.0 100.0 0.0 0 1  
```


#### ステータス フィードバックの取得:
&ensp;&ensp;  「xarm7_server.launch」を実行して実際の xArm ロボットに接続すると、ユーザーはトピック「xarm/xarm_states」にサブスクライブして、関節角度、TCP 位置、エラー/警告コードなどを含む現在のロボットの状態に関するフィードバック情報を得ることができます。コンテンツの詳細については、 RobotMsg.msgを参照してください。
  別のオプションは、「/joint_states」トピックをサブスクライブすることです。これはJointState.msgでレポートされますが、現在は「位置」フィールドのみが有効です; 「速度」は、隣接する 2 つの位置データに基づくフィルタリングされていない数値微分であり、「努力」フィードバックは、直接トルク センサーからではなく、電流ベースの推定値であるため、参考用です。パフォーマンスを考慮して、上記 2 つのトピックのデフォルトの更新レートは5Hzに設定されています。レポートの内容と頻度には他のオプションがあります。report_type 引数を参照してください。

#### Setting Tool Center Point Offset(only effective for xarm_api ROS service control):
&ensp;&ensp;ツール チップ ポイントのオフセット値は、サービス「/xarm/set_tcp_offset」を呼び出すことで設定できます。下の図を参照してください。このオフセット座標は、ベース フレームから (PI, 0, 0) のロール、ピッチ、ヨー回転で、フランジの中心に位置する既定のツール フレーム(フレーム B) に対して表されていることに注意してください (フレーム A)。   例えば：
  ![xArmFrames](https://user-images.githubusercontent.com/86779771/209673725-c1877d63-0cdd-4a2d-9bad-206d95fc360b.png)

```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp; これは、ツール フレームの位置オフセット (x = 0 mm、y = 0 mm、z = 20 mm) と、最初のツール フレーム (フレーム B のフレーム B写真）。このオフセットは、スタジオで設定されたデフォルト値と一致しない場合、xArm Stdudio によって上書きされる可能性があることに注意してください。ros サービス コントロールと一緒に使用する場合は、xArm studio で同じ TCP デフォルト オフセット設定を行うことをお勧めします。  

#### エラーのクリア:
&ensp;&ensp;コントローラーは、その後のコマンドの実行に影響を与えるエラーまたは警告を報告する場合があります。理由としては、電力損失、位置/速度制限違反、計画エラーなどが考えられます。クリアするには追加の介入が必要です。ユーザーは、トピック"xarm/xarm_states"のメッセージでエラー コードを確認できます。

ROSの復習です.以下のコマンドを打ってみると
```bash
$ rostopic list
```
```bash
/rosout
/rosout_agg
/xarm/controller_gpio_states
/xarm/joint_states
/xarm/sleep_sec
/xarm/xarm_gripper/gripper_action/cancel
/xarm/xarm_gripper/gripper_action/feedback
/xarm/xarm_gripper/gripper_action/goal
/xarm/xarm_gripper/gripper_action/result
/xarm/xarm_gripper/gripper_action/status
/xarm/xarm_states
```
これだけのトピックが配信されてることが分かります. 今回はこの中の/xarm/xarm_statesを見ます:

```bash
$ rostopic echo /xarm/xarm_states
```
```bash
---
header: 
  seq: 1622
  stamp: 
    secs: 1672214200
    nsecs: 963637319
  frame_id: ''
state: 2
mode: 0
cmdnum: 0
mt_brake: 63
mt_able: 63
err: 0
warn: 0
angle: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pose: [207.0, 3.6402626654155265e-14, 112.0, -3.1415927410125732, -0.0, 1.5407439555097887e-33]
offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
こんな感じにリアルタイムで表示されるはずです. 

以下のコマンド(moveit!)でxArmを起動している場合はもっとトピックが多いことでしょう.
```bash
roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.217 
```
&ensp;&ensp;errの表示がゼロ以外の場合、サービス"/xarm/clear_err"を呼び出すことにより、このエラー状態を取り除くことができます。
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;  Moveit! を使用している場合は、代わりに " /xarm/moveit_clear_err " を呼び出して、手動でモード 1 を再度設定する必要がないようにしてください。
```bash
$ rosservice call /xarm/moveit_clear_err
```
&ensp;&ensp;  このサービスを呼び出した後、「xarm/xarm_states」でエラー ステータスをもう一度確認してください。0 になっていれば、クリアは成功です。それ以外の場合は、エラー/例外が適切に解決されていないことを意味します。エラーのクリアに成功した場合は、ロボットの状態を 0に設定して、再び移動できるようにすることを忘れないでください。

# 4. Mode Change
&ensp;&ensp;xArm may operate under different modes depending on different controling methods. Current mode can be checked in the message of topic "xarm/xarm_states". And there are circumstances that demand user to switch between operation modes. 

### 4.1 Mode Explanation

&ensp;&ensp; ***Mode 0*** : xArm controller (Position) mode.  
&ensp;&ensp; ***Mode 1*** : External trajectory planner (position) mode.  
&ensp;&ensp; ***Mode 2*** : Free-Drive (zero gravity) mode.  
&ensp;&ensp; ***Mode 3*** : Reserved.  
&ensp;&ensp; ***Mode 4*** : Joint velocity control mode.  
&ensp;&ensp; ***Mode 5*** : Cartesian velocity control mode.  
&ensp;&ensp; ***Mode 6*** : Joint space online planning mode. (Firmware >= v1.10.0)  
&ensp;&ensp; ***Mode 7*** : Cartesian space online planning mode. (Firmware >= v1.11.0)  

&ensp;&ensp;***モード0***はシステム起動時のデフォルトであり、エラー発生時（衝突、過負荷、速度超過など）、自動的にモード0に切り替わります。また、[xarm_api](./xarm_api/)パッケージや[SDK](https://github.com/xArm-Developer/xArm-Python-SDK)のモーション機能では、xArmがMode 0で動作するよう要求されます。***モード1***は、Moveit!のような外部の軌道プランナーが、統合されたxArmプランナーをバイパスしてロボットを制御するためのものです。***モード2***はフリードライブ動作で、ロボットは重力補正モードに入りますが、モード2に設定する前に取り付け方向とペイロードが適切に設定されていることを確認してください。***モード4***はアームの速度を制御するモードです。***モード5***は直交座標系でのアームの直線的な速度を制御します。***モード6と7***は、それぞれ新しく生成された関節目標またはデカルト目標にリアルタイムで応答し、自動速度連続軌道再計画を行うためのモードです.

ってたくさん書いてるけど、Pythonで動かす場合はモード0しか対応してない？ようだからモード0使ってます(荒木). この姿勢できるかなーだったり、ティーチングで覚えさせたいとかあればモード2にすると良さそうかなくらいの理解です笑


### 4.2 モードの変更方法:  
&ensp;&ensp;  Moveit! の実行中に衝突やその他のエラーが発生した場合。計画された軌道、モードは安全のために 1 からデフォルトのモード 0 に自動的に切り替わり、ロボットの状態は 4 (エラー状態) に変わります。モードを 1 に戻さない限り、ロボットは Moveit コマンドを再度実行できません。次の手順に従って、Moveit コントロールを再度有効にします。  

&ensp;&ensp;(1)衝突の原因となっている物体が取り除かれていることを確認します。  
&ensp;&ensp;(2) エラーをクリアします。: 
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;(3) 目的のモード (たとえばモード 2) に切り替え、状態を 0 に設定して準備完了:
```bash
$ rosservice call /xarm/set_mode 2

$ rosservice call /xarm/set_state 0
```


# 5. Python control
&ensp;&ensp;ここまでは、ROSだけでアームを動かす方法を紹介しました. しかし、研究を行う上でカメラやハンドのモータから得たセンサデータを扱う以上、これらのコマンドをプログラムで引き出す必要があります. C++やPyhon様々な言語で動かせるみたいですけど、荒木が扱えるPythonで今回は紹介します.


### チュートリアル:
&ensp;&ensp; まずは、[チュートリアルファイル](https://github.com/hiroarkMani/xArm_manipulation/blob/dcd6d5f3bad6d83793fdf392e1be6583d0bcf932/xArm-Python-tutorial.py)をダウンロードしてみてください. このファイルはcatkin_ws/src下に置いてください.(正直どこでもいいですけど)

その後は以下のように操作してください.
```bash
$ cd catkin_ws/src
$ sudo chmod 755 xArm-Python-tutorial.py

$ cd ..
$ roslaunch xarm_gazebo xarm6_beside_table.launch
別ターミナルで
$ roslaunch xarm6_moveit_config xarm6_moveit_gazebo.launch
さらに別ターミナルで
$ python3 xArm-Python-tutorial.py
```

そうすると、実行できるはずです.
試しにmode"c"を選択してみると・・・
```bash
You select mode : c 
==========  Printing Xarm current pose: 
position: 
  x: 0.29999769778685975
  y: 0.40003553552068016
  z: 0.20003730083405907
orientation: 
  x: -0.9999040262992095
  y: -0.00038742999676679704
  z: 0.01383076476732452
  w: 0.0007057156499001798
==========  Printing Xarm current rpy:  ==========
[-3.1401700961440615, 0.027661855497863663, 0.0007946108753283219]
```
こんな表示が出て、現在のアームの姿勢を取得できるようになっています.

次に,mode"t"を選択してみると・・・
アームの先端が(x,y,z)=(300,400,200)の位置(単位は[mm])に移動するはずです.
ここで、mode"c"で位置を取得してみると、ずれていることがわかるはずです.

最後に、mode"h"を選択し、元の位置に戻してあげましょう.

[チュートリアルファイル](https://github.com/hiroarkMani/xArm_manipulation/blob/dcd6d5f3bad6d83793fdf392e1be6583d0bcf932/xArm-Python-tutorial.py)で使っているmoveitの関数はこの[サイト](https://robo-marc.github.io/moveit_documents/moveit_commander.html)を参考にしています. これからやる課題は、このページの関数を駆使してやってみましょう.

### 課題(1):キーボード入力で目標位置を与える
&ensp;&ensp; チュートリアルファイルに新しく関数を作ってキーボード入力で目標位置を与えてその位置に移動させるプログラムを書き、シミュレーション上で動作確認後、実機で動かしてみよ.

***注意事項***
   1. x,y,zの指令値は[m]単位です. 
   2. target_pose.orientation.*　の部分は弄らないようにしてください.
   3. 特に実機ではアームと机が干渉しないように、x,y,z軸の方向をしっかり把握し、入力を与えること.

### 課題(2):干渉チェック
&ensp;&ensp; PlanningScene上でboxを邪魔な位置に生成してみた上で(1)の位置に動作させてみよう.
```bash
box(キムワイプ)のサイズ指定: 130*120*95[mm]
```
コメントアウトしている
```bash
psi = moveit_commander.PlanningSceneInterface()
```
↑　これがヒント！！あとはこの[サイト](https://robo-marc.github.io/moveit_documents/moveit_commander.html)を活用して頑張ってみよう.


### 課題(2):干渉チェック
&ensp;&ensp; PlanningScene上でboxを邪魔な位置に生成してみた上で(1)の位置に動作させてみよう.
```bash
box(手指消毒液)のサイズ指定: 85*85*260[mm]
```
コメントアウトしている
```bash
psi = moveit_commander.PlanningSceneInterface()
```
↑　これがヒント！！あとはこの[サイト](https://robo-marc.github.io/moveit_documents/moveit_commander.html)を活用して頑張ってみよう.


### 課題(3):滑らかに円軌道を描け
&ensp;&ensp; z=0.300 で適当に円軌道を素速く描け.
試しに、
```bash
target_pose.x=???
xarm.set_pose_target(target_pose)
xarm.go()
```
の形を繰り返して、動作させてみよう.何かおかしな点が見つかるはず…
それを解決してみよう.

ヒント：動かす関数は何もgo()だけではない.



### 課題(4):(以下任意) ペンアタッチメントをエンドエフェクタとして付けてみよう
&ensp;&ensp;  URDFファイルを編集し、ペンアタッチメントをエンドエフェクタとして付けてみる.


### 課題(5): イニシャルを描け
&ensp;&ensp;  (4)の環境のまま,ペンを付けて自分のイニシャル文字を書いてみせよ.


