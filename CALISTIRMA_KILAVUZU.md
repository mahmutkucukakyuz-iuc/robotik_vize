# rk6 Çalıştırma Kılavuzu

Bu kılavuz iki senaryonun nasıl çalıştırılacağını ve çalışırken ikinci bir terminalden nelerin kontrol edilebileceğini anlatır.

| Senaryo | Ne gösterir | Paketler |
|---|---|---|
| **1. Ödevin istedikleri** | Kol modeli, sensörler (kamera, IMU), gripper, topic/action ile kontrol, FK hata analizi, Home → K1 → K2 → K3 → Home demosu | `rk6_description`, `rk6_control` |
| **2. Ekstra** | Ters kinematik, bilek kamerasıyla küp bulma, al-bırak, küpleri üst üste dizme | `rk6_manipulation` (+ yukarıdakiler) |

İki senaryo **aynı anda çalıştırılmaz**: birini kapatıp (Ctrl+C) diğerini başlatın. Ödev senaryosu ekstra pakete bağlı değildir.

---

## 0. Hazırlık

Kurulum ve ilk derleme için: [README — Kurulum](README.md#kurulum).

### Terminaller
Komutlar Ubuntu terminalinde çalıştırılır (WSL2 kullanıyorsanız PowerShell'de `wsl -d Ubuntu-26.04`). Her senaryo için **en az iki** terminal gerekir:

- **Terminal 1:** simülasyon (Gazebo) — sürekli açık kalır
- **Terminal 2:** demo, analiz ve kontrol komutları

`kurulum_wsl.sh` ile kurulum yapıldıysa `~/.bashrc` ROS ortamını, çalışma alanını (`~/rt_ws`) ve ekran kartı ayarlarını her yeni terminalde otomatik yükler. Ayarlar yüklenmemiş görünüyorsa:

```bash
source ~/.bashrc
```

### Kodda değişiklikten sonra yeniden derlemek

```bash
source /opt/ros/lyrical/setup.bash && cd ~/rt_ws && colcon build --symlink-install && source install/setup.bash
```

Depo `~/rt_ws/src` dışında bir klasördeyse (`kurulum_wsl.sh` paketleri oradan `~/rt_ws/src`'ye kopyalar), derlemeden önce paketleri yeniden kopyalayın: `rsync -a --delete <depo_klasoru>/ros2_ws/src/ ~/rt_ws/src/`

Derleme sonunda `Summary: 3 packages finished` görünmelidir. Yeni bir paket eklendiyse yeni terminal açın.

---

## 1. Senaryo 1 — Ödevin istedikleri

### 1.1 (İsteğe bağlı) Sadece modeli görmek: RViz
Gazebo olmadan, kaydırıcılarla eklemleri oynatarak modeli kontrol etmek için:

```bash
ros2 launch rk6_description display.launch.py
```

Açılan **Joint State Publisher** penceresinde kaydırıcıları oynatınca RViz'deki kol hareket eder. Kapatmak için Ctrl+C.

### 1.2 Simülasyonu başlatmak (Terminal 1)

```bash
ros2 launch rk6_description gazebo.launch.py rviz:=true
```

Gazebo ve RViz pencereleri açılır. Dünyada kol ve kolun erişemeyeceği uzaklıkta, kameranın görebileceği renkli nesneler vardır (küp yoktur).

Argümanlar:

| Argüman | Varsayılan | Anlamı |
|---|---|---|
| `rviz` | `false` | `true` ise RViz de açılır |
| `ee_pose` | `false` | `true` ise `/ee_pose` yayıncısı da başlar (demo/analiz launch'ları zaten başlatır; ikisini birlikte kullanmayın) |
| `world` | vize dünyası | Başka bir dünya dosyası |

### 1.3 Simülasyonun hazır olduğunu kontrol etmek (Terminal 2)

```bash
ros2 control list_controllers
```

Üç kontrolcü de **active** olmalı:

```
gripper_controller      joint_trajectory_controller/JointTrajectoryController  active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

### 1.4 Hareket demosu: Home → K1 → K2 → K3 → Home (Terminal 2)

```bash
ros2 launch rk6_control demo.launch.py
```

Bu tek komut üç node başlatır:

| Node | Görevi |
|---|---|
| `ee_pose_publisher` | Uç efektörün tabana göre konumunu `/ee_pose`'a yayınlar |
| `visualizer` | **İki pencere açar:** eklem açılarının canlı grafiği ve bilek kamerası görüntüsü; grafiğin altında uç efektör konumu yazar |
| `motion_sequence` | 5 saniye sonra kolu sırayla K1, K2, K3'e götürüp home'a döndürür; her durakta gripper açılıp kapanır |

Argümanlar: `motion_delay:=5.0` (hareket başlamadan bekleme), `show_camera:=false` (kamera penceresini kapatır). Demo bitince pencereler açık kalır; Ctrl+C ile kapatın.

### 1.5 FK hata analizi: T1–T10 (Terminal 2)

```bash
ros2 launch rk6_control fk_analysis.launch.py
```

Kol 10 test konfigürasyonunu sırayla dolaşır. Her birinde DH ile hesaplanan konum, ROS'tan (`/ee_pose`) ve Gazebo'dan (`/gz_link_poses`) okunan konumla karşılaştırılır. Sonunda tablo, ortalama ve standart sapma ekrana basılır ve şuraya kaydedilir:

```
~/rk6_results/fk_error_<tarih>.csv
~/rk6_results/fk_error_<tarih>.md
```

Analiz bitince launch kendiliğinden kapanmaz; tabloyu gördükten sonra Ctrl+C ile kapatın (kapanırken çıkan uyarılar zararsızdır).

### 1.6 Eklemleri komut satırından hareket ettirmek (Terminal 2)

Kol, **action** ile (sonucu bekler, geri bildirim verir). Örnek K3 = (0°, −20°, 100°, 0°, 70°, 0°):

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, -0.3491, 1.7453, 0.0, 1.2217, 0.0], time_from_start: {sec: 3}}]}}" --feedback
```

Kol, **topic** ile. Örnek K1 = (45°, 30°, 60°, 0°, 60°, 0°):

```bash
ros2 topic pub --once -w 1 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.7854, 0.5236, 1.0472, 0.0, 1.0472, 0.0], time_from_start: {sec: 3}}]}"
```

Home'a dönmek için `positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` gönderin. (`-w 1` mesajın kontrolcü abone olmadan gönderilip kaybolmasını önler; onsuz `--once` bazen kolu hareket ettirmez.)

Gripper (metre; 0 = kapalı, 0.0235 = açık; eklem sınırı −0.002 … 0.025):

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [gripper_left_finger_joint, gripper_right_finger_joint], points: [{positions: [0.0235, 0.0235], time_from_start: {sec: 1}}]}}"
```

### 1.7 Node'ları tek tek çalıştırmak (isteğe bağlı)
`demo.launch.py` yerine her biri ayrı terminalde:

```bash
ros2 run rk6_control ee_pose_publisher --ros-args -p use_sim_time:=true
ros2 run rk6_control visualizer --ros-args -p use_sim_time:=true
ros2 run rk6_control motion_sequence --ros-args -p use_sim_time:=true
```

---

## 2. Senaryo 2 — Ekstra: manipülasyon (ters kinematik + kamera)

Bu senaryo ödev kapsamı dışındaki ekstra çalışmadır. Önce ödev simülasyonunu Terminal 1'de **Ctrl+C** ile kapatın.

### 2.0 Tek komutla: simülasyon + tara ve diz (en kolayı)

```bash
ros2 launch rk6_manipulation stack_demo.launch.py
```

Küplü dünyayı ve Gazebo'yu açar; 10 saniye sonra (kontrolcüler hazır olunca) `scan_and_stack`'i kamera penceresiyle birlikte başlatır. Yaklaşık 3 dakikada (tarama ~80 s) üç küp taranır ve dizilir. Görev bitince simülasyon açık kalır; tekrar denemek için Ctrl+C ile kapatıp komutu yeniden çalıştırın (küpler yerine döner).

| Argüman | Varsayılan | Anlamı |
|---|---|---|
| `rviz` | `false` | RViz de açılsın |
| `run_task` | `true` | `false` ise sadece simülasyon açılır (2.1 ile aynı) |
| `show_camera` | `true` | "rk6 bilek kamerasi" penceresi |
| `start_delay` | `10.0` | Görev başlamadan önce bekleme [s]; bilgisayar yavaşsa artırın |

Aşağıdaki 2.1–2.4 adımları aynı işleri parça parça yapmak içindir.

### 2.1 Küplü dünyayı başlatmak (Terminal 1)

```bash
ros2 launch rk6_manipulation pick_world.launch.py
```

RViz de istenirse `rviz:=true` ekleyin. Dünyada zeminde kırmızı, mavi ve yeşil üç küp (3 cm) vardır. Kontrolcüleri 1.3'teki komutla kontrol edin.

### 2.2 Kamerayla tara ve üst üste diz (Terminal 2)

```bash
ros2 run rk6_manipulation scan_and_stack --ros-args -p use_sim_time:=true
```

Ne olur:

1. **"rk6 bilek kamerasi"** penceresi açılır: canlı görüntü, algılanan küpler (sarı çerçeve) ve üstte o anki adım.
2. Kol, gripper aşağı ve biraz dışarı bakacak şekilde etrafı tarar (~80 s). İki halka vardır: önce yakın bant, sonra uzak bant. Her halkada −125°…+125° arasında 15 noktada durur, toplam 30 durak. Çalışma alanındaki her nokta en az 2 görüntüde görünür.
3. Bulunan küpler terminale yazılır (konum ve açı).
4. Küpler sırayla alınıp robotun önüne, (0.25, 0.00) noktasına üst üste dizilir; sonra kol home'a döner.

Pencereyi `q` ile kapatabilirsiniz; görev sürer. İşaretli tarama görüntüleri `~/rk6_results/scan/` klasörüne kaydedilir.

Sık kullanılan parametreler (`-p isim:=deger` ile):

| Parametre | Varsayılan | Anlamı |
|---|---|---|
| `colors` | `"[red, green, blue]"` | Dizme sırası (ilk renk en altta) |
| `stack_xy` | `"[0.25, 0.0]"` | Kulenin yeri (base_link'e göre, metre) |
| `show_camera` | `true` | Kamera penceresi |
| `scan_yaws_deg` | −125, −115, −100, −80…80 (20'şer), 100, 115, 125 | Tarama açıları (her halkada) |
| `scan_radius` / `scan_z` / `scan_tilt_deg` | `[0.14, 0.14]` / `[0.125, 0.18]` / `[10, 30]` | Halka başına tcp yarıçapı, yüksekliği ve eğimi |

Örnek, maviyi en alta koymak:

```bash
ros2 run rk6_manipulation scan_and_stack --ros-args -p use_sim_time:=true -p colors:="[blue, red, green]"
```

**Kameranın gerçekten gördüğünü denemek:** Gazebo'da küpü seçip üstteki **Translate** (taşı) ya da **Rotate** (döndür) aracıyla başka bir yere koyun ve komutu tekrar çalıştırın. Kolun erişebildiği alan tabandan yaklaşık 0.20–0.40 m uzaklıktır. Tarama 0.18–0.40 m arasını en az 2 görüntüyle kaplar. Robotun önü ve yanları taranır (±133°); arkası taranmaz.

Küpleri başlangıç yerlerine döndürmek için Terminal 1'de simülasyonu yeniden başlatın.

### 2.3 Ters kinematikle bir noktaya gitmek (Terminal 2)

Gripper'ın kavrama noktasını (parmakların ortası) verilen noktaya götürür; koordinatlar base_link'e göre metredir (robot doğrudan zeminde durur; zemin `z = 0`):

```bash
ros2 run rk6_manipulation ik_goto --ros-args -p use_sim_time:=true -p x:=0.30 -p y:=0.10 -p z:=0.05
```

Terminalde bulunan eklem açıları ve FK ile doğrulaması yazar. Sadece hesaplayıp hareket ettirmemek için `-p dry_run:=true` ekleyin. Erişilemeyen bir nokta verilirse kol hareket etmez ve uyarı yazar.

### 2.4 Bilinen konumdan al-bırak (Terminal 2)

Kamerasız sürüm: küp konumları parametre olarak verilir (`[x, y, açı_derece, ...]`). Varsayılan, küplü dünyadaki kırmızı ve mavi küpü taşır:

```bash
ros2 run rk6_manipulation pick_place --ros-args -p use_sim_time:=true
```

Örnek, yeşil küpü (0.30, 0.10)'a taşımak (varsayılan örnek kırmızıyı (0.28, 0.12)'ye bıraktığı için ikisini art arda çalıştıracaksanız önce simülasyonu yeniden başlatın, yoksa küpler çarpışır):

```bash
ros2 run rk6_manipulation pick_place --ros-args -p use_sim_time:=true -p picks:="[-0.06, -0.29, 9.74]" -p places:="[0.30, 0.10, 0.0]"
```

---

## 3. Kontrol ve izleme komutları (her iki senaryoda, Terminal 2 veya 3)

| Ne kontrol edilir | Komut |
|---|---|
| Kontrolcüler aktif mi | `ros2 control list_controllers` |
| Tüm topic'ler | `ros2 topic list` |
| Eklem açıları [rad] | `ros2 topic echo /joint_states` |
| Uç efektör konumu (tabana göre) | `ros2 topic echo /ee_pose` *(ee_pose_publisher çalışıyorken)* |
| Aynı bilgi doğrudan TF'ten | `ros2 run tf2_ros tf2_echo base_link tool0` |
| Gazebo'nun ölçtüğü link konumları | `ros2 topic echo --once /gz_link_poses` |
| IMU verisi | `ros2 topic echo /imu` |
| Kamera yayın hızı | `ros2 topic hz /wrist_camera/image_raw` |
| Kamera görüntüsü (ayrı pencere) | `ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw` |
| scan_and_stack'in işaretli görüntüleri | `ros2 run rqt_image_view rqt_image_view /scan_and_stack/detections` |
| Node/topic diyagramı | `ros2 run rqt_graph rqt_graph` |
| TF ağacı (PDF üretir) | `ros2 run tf2_tools view_frames` |

Kontrol değeri: home konumunda (kol dimdik yukarı) `/ee_pose` yaklaşık **(0.000, −0.001, 0.792) m** okunmalıdır (FK: (0, −0.0008, 0.79213) m).

---

## 4. Sorun giderme

| Belirti | Çözüm |
|---|---|
| RViz/Gazebo açılırken "Invalid parentWindowHandle" | Terminal eski ayarla açılmış: `source ~/.bashrc` ya da yeni terminal |
| Kontrolcülerden biri `active` değil | Simülasyonu yeniden başlatın; hâlâ olmazsa Terminal 1'deki hata satırlarına bakın |
| `ros2 run rk6_manipulation ...` "package not found" | Yeni terminal açın ya da `source ~/rt_ws/install/setup.bash` |
| Gazebo kapandıktan sonra yeniden açılmıyor / takılıyor | Arkada kalan süreçleri kapatın: `pkill -f gz-sim-main` (yalnızca Ctrl+C sonrası süreç kaldıysa) |
| Küpler dağıldı / tekrar denemek istiyorum | Senaryo 2 simülasyonunu yeniden başlatın |
| `/ee_pose` iki kere yayınlanıyor | `gazebo.launch.py ee_pose:=true` ile `demo/fk_analysis` launch'larını birlikte kullanmayın |
| Gazebo yavaş | `nvidia-smi` ile ekran kartının kullanıldığını kontrol edin; `glxinfo -B` çıktısında "NVIDIA" görünmeli |
