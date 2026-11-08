# rk6: 6 DoF robot kol (ROS 2 Lyrical + Gazebo Jetty)

BLMU7619 Robot Teknolojileri vize projesi. `rk6` kendi tasarımım olan bir koldur:

- 6 döner eklem, geometrik sınıfı **R ⊢ R ∥ R** (articulated).
- URDF/Xacro ile tanımlı ve Gazebo Jetty'de simüle ediliyor.
- `gz_ros2_control` ile topic veya action üzerinden sürülüyor.
- Bilek kamerası ve IMU içeriyor.

Bütün birimler SI'dır: açılar radyan, uzunluklar metre.

## Paketler

| Paket | Tip | İçerik |
|---|---|---|
| `rk6_description` | ament_cmake | URDF/Xacro, kontrolcü YAML, köprü YAML, `gazebo.launch.py`, `display.launch.py`, RViz ayarı, dünya dosyası |
| `rk6_control` | ament_python | `ee_pose_publisher`, `motion_sequence`, `visualizer`, `fk_error_analysis`, `kinematics.py` (DH düz kinematik) |
| `rk6_manipulation` | ament_python | **Ek senaryo:** `ik.py` (ters kinematik), `ik_goto`, `pick_place`, `scan_and_stack` (kamera ile küp bulma ve üst üste dizme), küplü dünya `worlds/rk6_pick_world.sdf` |

### İki senaryo

| Senaryo | Simülasyon | Kapsam |
|---|---|---|
| **1. Ödevin istedikleri (vize)** | `ros2 launch rk6_description gazebo.launch.py` (küpsüz dünya) | Kol, sensörler, gripper, topic/action kontrolü, FK hata analizi, Home→K1→K2→K3 demosu (`rk6_control`) |
| **2. Ekstra: manipülasyon** | `ros2 launch rk6_manipulation pick_world.launch.py` (küplü dünya) | Ters kinematik, kamera ile küp algılama, al-bırak, üst üste dizme (`rk6_manipulation`) |

Birinci senaryo ikinci paketi kullanmaz; `rk6_manipulation` olmadan da derlenir ve çalışır.

## 1. Gereksinimler (Ubuntu 26.04 + ROS 2 Lyrical)

Aşağıdaki komutlar Gazebo Jetty'nin OSRF deposundan kurulmuş olduğunu varsayar (`gz sim --version` ile kontrol edin).

```bash
sudo apt install ros-lyrical-ros-gz ros-lyrical-gz-ros2-control \
  ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
  ros-lyrical-xacro ros-lyrical-robot-state-publisher \
  ros-lyrical-joint-state-publisher-gui ros-lyrical-rviz2 \
  ros-lyrical-cv-bridge ros-lyrical-rqt-image-view ros-lyrical-tf2-tools \
  python3-numpy python3-matplotlib python3-opencv
```

## 2. Derleme

```bash
mkdir -p ~/rt_ws/src && cd ~/rt_ws/src
git clone https://github.com/mahmutkucukakyuz-iuc/robotik_vize.git
cd ~/rt_ws
rosdep install --from-paths src --ignore-src -r -y              # isteğe bağlı
colcon build --symlink-install
source install/setup.bash        # her yeni terminalde (veya ~/.bashrc'ye ekleyin)
```

> **Not:** Çalışma alanını yolunda boşluk olmayan bir klasörde derleyin (ör. `~/rt_ws`). Bulut eşitleme ya da VM paylaşımlı klasörleri gibi boşluk içeren yollar bazı araçlarda sorun çıkarabilir. Launch dosyaları xacro'yu Python API'siyle çağırdığı için `robot_description` üretimi boşluklu yolda da çalışır, ama `~/rt_ws` önerilir.

## 3. Çalıştırma

```bash
# Model kontrolü: yalnızca RViz ve eklem kaydırıcıları
ros2 launch rk6_description display.launch.py

# Simülasyon: Gazebo Jetty, robot_state_publisher, spawn, kontrolcüler ve ros_gz_bridge
ros2 launch rk6_description gazebo.launch.py
# (isteğe bağlı) RViz ve /ee_pose yayıncısı ile birlikte:
ros2 launch rk6_description gazebo.launch.py rviz:=true ee_pose:=true
```

Gazebo açıkken ikinci bir terminalde kendi node'larımızı launch dosyalarıyla başlatın (önerilen yol; `use_sim_time` otomatik olarak `true`):

```bash
# Hareket görevi: ee_pose_publisher + visualizer + motion_sequence (Home -> K1 -> K2 -> K3 -> Home)
ros2 launch rk6_control demo.launch.py

# Hata analizi: ee_pose_publisher + fk_error_analysis (T1..T10), sonuçlar ~/rk6_results/*.csv ve *.md
ros2 launch rk6_control fk_analysis.launch.py
```

> `gazebo.launch.py ee_pose:=true` ile birlikte `demo.launch.py` ya da `fk_analysis.launch.py` çalıştırmayın; ikisi de `ee_pose_publisher` başlatır.

Gazebo duraklatılmış açılırsa Play'e basın. Kontrolcülerin durumunu şöyle kontrol edin:

```bash
ros2 control list_controllers      # joint_state_broadcaster, arm_controller ve gripper_controller -> active olmalı
ros2 topic list
```

Node'ları tek tek çalıştırmak isterseniz (her biri ayrı terminalde; simülasyon `/clock` ile çalıştığı için `use_sim_time:=true` gereklidir):

```bash
ros2 run rk6_control ee_pose_publisher --ros-args -p use_sim_time:=true   # /ee_pose (TF base_link -> tool0)
ros2 run rk6_control visualizer        --ros-args -p use_sim_time:=true   # /joint_states grafiği ve kamera penceresi
ros2 run rk6_control motion_sequence   --ros-args -p use_sim_time:=true   # Home -> K1 -> K2 -> K3 -> Home
ros2 run rk6_control fk_error_analysis --ros-args -p use_sim_time:=true   # T1..T10: FK ile Gazebo karşılaştırması
```

## 4. Eklemleri komut satırından hareket ettirme

Eklem isimleri `joint1` … `joint6`'dır. Hedef değerler radyan cinsindendir.

**Topic ile (JointTrajectory).** Örnek hedef K1 = (45°, 30°, 60°, 0°, 60°, 0°):

```bash
ros2 topic pub --once -w 1 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
  points: [{positions: [0.7854, 0.5236, 1.0472, 0.0, 1.0472, 0.0],
            time_from_start: {sec: 3, nanosec: 0}}]}"
```

**Action ile (FollowJointTrajectory, feedback'li).** Örnek hedef K3 = (0°, −20°, 100°, 0°, 70°, 0°):

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
  points: [{positions: [0.0, -0.3491, 1.7453, 0.0, 1.2217, 0.0],
            time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback
```

Home konumuna dönmek için `positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` gönderin.

**Gripper (metre, 0 = kapalı, 0.0235 = açık; eklem sınırı −0.002 … 0.025).** Gripper DoF sayısına dahil değildir; `gripper_controller` ile ayrı sürülür:

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: [gripper_left_finger_joint, gripper_right_finger_joint],
  points: [{positions: [0.0235, 0.0235], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

Kapatmak için `positions: [0.0, 0.0]` gönderin. Gripper'sız model gerekirse xacro'ya `gripper:=false` verilir.

## 4A. Ekstra (Senaryo 2): ters kinematik, kamera ile algılama ve üst üste dizme

Tek komutla (simülasyon + 10 s sonra kamerayla tara ve diz; `run_task:=false` ile sadece simülasyon):

```bash
ros2 launch rk6_manipulation stack_demo.launch.py
```

Parça parça çalıştırmak için önce küplü dünyayı açın (vize simülasyonu açıksa Ctrl+C ile kapatın):

```bash
ros2 launch rk6_manipulation pick_world.launch.py
```

**Kamerayla tara ve diz.** Kol bilek kamerası yere bakacak şekilde etrafı tarar, renkli küpleri bulur, hepsini robotun önüne (0.25, 0.00) üst üste dizer. Küp konumları robota verilmez; kameradan hesaplanır. Çalışırken **"rk6 bilek kamerasi"** penceresi açılır: canlı görüntü, algılanan küpler ve o anki adım görünür (pencereyi `q` ile kapatabilirsiniz, görev sürer).

```bash
ros2 run rk6_manipulation scan_and_stack --ros-args -p use_sim_time:=true
```

İşaretli tarama görüntüleri `~/rk6_results/scan/` klasörüne kaydedilir ve `/scan_and_stack/detections` topic'inde yayınlanır:

```bash
ros2 run rqt_image_view rqt_image_view /scan_and_stack/detections
```

Küplerin kamerayla bulunduğunu denemek için onları Gazebo'da elle (Translate/Rotate aracıyla) başka yerlere taşıyıp komutu tekrar çalıştırabilirsiniz (tarama tabandan yaklaşık 0.18–0.40 m arasını, robotun önünü ve yanlarını kapsar; arkası taranmaz).

**Ters kinematik ile noktaya gitme.** `rk6_manipulation/ik.py` sayısal ters kinematik çözücüdür (Jacobian, sönümlü en küçük kareler). Hedef, parmakların ortasındaki `gripper_tcp` noktasıdır; koordinatlar `base_link`'e göre metre cinsindendir (robot doğrudan zeminde; zemin `z = 0`).

```bash
ros2 run rk6_manipulation ik_goto --ros-args -p use_sim_time:=true -p x:=0.30 -p y:=0.10 -p z:=0.05
```

Yalnızca çözümü görmek için `-p dry_run:=true`, tam yönelim vermek için `-p mode:=rpy -p roll_deg:=.. -p pitch_deg:=.. -p yaw_deg:=..` kullanın.

**Bilinen konumlardan al-bırak** (kamerasız; konumlar parametre olarak verilir, varsayılanlar küplü dünyadaki kırmızı ve mavi küptür):

```bash
ros2 run rk6_manipulation pick_place --ros-args -p use_sim_time:=true
ros2 run rk6_manipulation pick_place --ros-args -p use_sim_time:=true \
  -p picks:="[-0.06, -0.29, 9.74]" -p places:="[0.30, 0.10, 0.0]"
```

Küpleri başlangıç yerlerine döndürmek için simülasyonu yeniden başlatın.

## 5. Sensör ve durum verisini okuma

```bash
ros2 topic echo /joint_states                 # eklem açıları [rad]
ros2 topic echo /ee_pose                      # uç-efektör pozu, frame_id: base_link [m]
ros2 run tf2_ros tf2_echo base_link tool0     # aynı bilgi, doğrudan TF'ten
ros2 topic echo --once /gz_link_poses         # Gazebo'nun ölçtüğü link pozları (model çerçevesine göre; tool0 dahil)
ros2 topic echo /imu                          # IMU: açısal hız ve doğrusal ivme
ros2 topic hz /wrist_camera/image_raw
ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw   # kamera görüntüsü
ros2 run rqt_graph rqt_graph                  # node/topic diyagramı
ros2 run tf2_tools view_frames                # TF ağacı (frames_*.pdf)
```

Kontrol değeri: home konumunda ($\theta=0$, kol dimdik yukarı) `/ee_pose` yaklaşık **(0.000, −0.001, 0.792) m** okunmalıdır (FK: (0, −0.0008, 0.79213) m).

## 6. Topic / action özeti

| İsim | Tip |
|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` (metre) |
| `/ee_pose` | `geometry_msgs/msg/PoseStamped` (`base_link`) |
| `/gz_link_poses` | `tf2_msgs/msg/TFMessage` (Gazebo PosePublisher `/rk6/pose` → ros_gz_bridge) |
| `/wrist_camera/image_raw` | `sensor_msgs/msg/Image` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/clock` | `rosgraph_msgs/msg/Clock` |

## 7. Sorun giderme

- **Kamera veya IMU yayın yapmıyor:** Dünya dosyasında `gz-sim-sensors-system` ve `gz-sim-imu-system` plugin'lerinin olduğundan emin olun.
- **Gazebo kapandıktan sonra süreç kaldıysa:** `pgrep -af gz` ile bulun, `pkill -f gz-sim-main` ile kapatın.
- **Kontrolcü `inactive` görünüyorsa:** `ros2 run controller_manager spawner arm_controller` ile yeniden başlatın.

## Rapor

Rapor depoda `rapor/4801260209.pdf` (kaynağı `rapor/rapor_taslak.md`) olarak bulunur. FK modeli, DH tablosu ve hata analizi raporda anlatılmıştır.
