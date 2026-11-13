# NOVA AUV — Sualtı Araç Simülasyon Platformu

**ROS2 Humble · Gazebo Classic · Ubuntu 22.04**  
**6-DoF · 8 Thruster · BlueRobotics T200**

---

## Görseller

| RViz — Robot Modeli | RViz — TF Ağacı |
|:-------------------:|:---------------:|
| ![RViz Robot](docs/rviz_robot.png) | ![RViz TF](docs/rviz_tf.png) |

| RViz — TF Çerçeveleri | Gazebo — Sualtı Simülasyonu |
|:---------------------:|:---------------------------:|
| ![TF Frames](docs/rviz_tf_only.png) | ![Gazebo](docs/gazebo_sim.png) |

---

## İçindekiler

- [Proje Hakkında](#proje-hakkında)
- [Sistem Mimarisi](#sistem-mimarisi)
- [TF Ağacı](#tf-ağacı)
- [ROS2 Topic Haritası](#ros2-topic-haritası)
- [Araç Özellikleri](#araç-özellikleri)
- [Thruster Konfigürasyonu](#thruster-konfigürasyonu)
- [Kurulum](#kurulum)
- [Kullanım](#kullanım)
- [Paket Yapısı](#paket-yapısı)

---

## Proje Hakkında

NOVA, Gazebo Classic tabanlı fizik simülasyonunda çalışan 8 thruster'lı, 6-DoF otonom sualtı aracı (AUV) platformudur.

**Temel özellikler:**
- Kuadratik hidrodinamik sürükleme (gerçekçi su direnci modellemesi)
- 6×8 Thrust Allocation Matrix (TAM) ile tersinir thruster çözümü
- Derinlik / başlık / ilerleme için PID kontrolcü
- Klavye ile 6-DoF manuel kontrol
- Modüler paket yapısı (sensör ve otonom katmanlar genişletilebilir)

---

## Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 HUMBLE STACK                        │
│                                                                 │
│  ┌────────────┐    ┌──────────────────┐    ┌─────────────────┐ │
│  │  teleop.py │    │  pid_controller  │    │  nova_autonomy  │ │
│  │  (klavye)  │    │  (derinlik/başlık│    │  (gelecek faz)  │ │
│  └─────┬──────┘    └────────┬─────────┘    └────────┬────────┘ │
│        │                    │                        │          │
│        └────────────────────┴────────────────────────┘          │
│                             │                                   │
│                      /cmd_vel (Twist)                           │
│                             │                                   │
│                    cmd_vel_to_wrench                            │
│                             │                                   │
│                   /control/wrench_cmd                           │
│                      ┌──────┴──────┐                            │
│                      ▼             ▼                            │
│            thruster_allocator  hydrodynamics                   │
│                      │             │                            │
│              /thrusters/commands   │  (sürükleme hesaplama)    │
│                                    ▼                            │
│                             gazebo_bridge                       │
│                                    │                            │
└────────────────────────────────────┼────────────────────────────┘
                                     │
                           /nova/gazebo_ros_force
                                     │
                              ┌──────▼──────┐
                              │   GAZEBO    │
                              │  CLASSIC    │
                              │ underwater  │
                              │   .world    │
                              └─────────────┘
```

---

## TF Ağacı

```
world
 └── base_link          (robot_state_publisher — URDF'den)
      ├── thruster_1_link    (ön sağ yatay  — +45°)
      ├── thruster_2_link    (ön sol yatay  — +135°)
      ├── thruster_3_link    (arka sol yatay — +45°)
      ├── thruster_4_link    (arka sağ yatay — -45°)
      ├── thruster_5_link    (ön sağ dikey)
      ├── thruster_6_link    (ön sol dikey)
      ├── thruster_7_link    (arka sağ dikey)
      └── thruster_8_link    (arka sol dikey)
```

> Canlı TF görüntülemek için: `ros2 run tf2_tools view_frames`

---

## ROS2 Topic Haritası

```
Kullanıcı / Otonom Node
  └─► /cmd_vel  (Twist)
           │
           ▼
  cmd_vel_to_wrench  (20 N·s/m · lineer, 3 N·m·s/rad · açısal)
           │
           └─► /control/wrench_cmd  (Wrench)
                    ├──────────────────────────────────────────────┐
                    ▼                                              ▼
          thruster_allocator                               hydrodynamics
          6×8 TAM psödo-tersi                      ρ·Cd·A·v·|v| sürükleme
                    │                                              │
                    ▼                                              ▼
          /thrusters/commands                       /nova/drag_wrench (tanı)
          (8× PWM, 1100-1900 μs)                              +
                                                   /nova/gazebo_ros_force
                                                              │
                    gazebo_bridge ◄────────────────────────────┘
                           │
                           └─► /nova/gazebo_ros_force ──► Gazebo fizik motoru

Sensörler (gelecek faz):
  /pressure/data   (PressureData) ──► pid_controller
  /dvl/data        (DvlData)      ──► pid_controller
  /imu/data        (ImuData)      ──► pid_controller + otonom

Setpoint:
  /setpoint/depth   (Float64) ──► pid_controller
  /setpoint/heading (Float64) ──► pid_controller
  /setpoint/surge   (Float64) ──► pid_controller
```

---

## Araç Özellikleri

### Mekanik

| Parametre | Değer |
|-----------|-------|
| Uzunluk (X) | ~0.47 m |
| Genişlik (Y) | 0.625 m |
| Yükseklik (Z) | 0.35 m |
| Kütle | 15 kg |
| Yüzdürme | Nötr (yerçekimi = 0) |

### Atalet Tensörü

| Eksen | Değer |
|-------|-------|
| Ixx (Yalpa) | 0.466 kg·m² |
| Iyy (Yunuslama) | 0.801 kg·m² |
| Izz (Sapma) | 0.641 kg·m² |

### Hidrodinamik Sürükleme

Kuadratik model: `F_drag = -0.5 · ρ · Cd · A · v · |v|`  
Su yoğunluğu (tuzlu su): **ρ = 1025 kg/m³**

| Eksen | Cd × A | Yorum |
|-------|--------|-------|
| İleri (X) | 0.0175 m² | Aerodinamik gövde, düşük direnç |
| Yan (Y) | 0.175 m² | Yan profil, yüksek direnç |
| Düşey (Z) | 0.250 m² | Üst/alt profil, en yüksek direnç |
| Yalpa (Rx) | 0.012 | Dönüşel sönümleme |
| Yunuslama (Ry) | 0.018 | Dönüşel sönümleme |
| Sapma (Rz) | 0.018 | Dönüşel sönümleme |

### PID Kontrolcü Parametreleri

| Kontrolcü | Kp | Ki | Kd | Frekans |
|-----------|----|----|-----|---------|
| Derinlik (Z) | 20.0 | 0.1 | 5.0 | 50 Hz |
| Başlık (Yaw) | 15.0 | 0.05 | 3.0 | 50 Hz |
| İlerleme (X) | 10.0 | 0.0 | 2.0 | 50 Hz |

---

## Thruster Konfigürasyonu

8× **BlueRobotics T200** — Max ±40 N, PWM: 1100–1900 μs

```
Üstten görünüm (yatay thruster'lar):

          ► (İleri)
       T2 ╲       ╱ T1
          ╲     ╱
    ┌──────╲───╱──────┐
    │       ╲ ╱       │
    │        X        │  ← gövde merkezi
    │       ╱ ╲       │
    └──────╱───╲──────┘
       T3 ╱       ╲ T4
          ► (İleri, 45° yönlü)

Yan görünüm (dikey thruster'lar):
  T5 (ön sağ) ↕   T6 (ön sol) ↕
  T7 (arka sağ) ↕  T8 (arka sol) ↕
```

### Thruster Pozisyonları (base_link'e göre)

**Yatay (Surge / Sway / Yaw):**

| Thruster | Pozisyon (x, y, z) m | Yaw açısı | Rol |
|----------|----------------------|-----------|-----|
| T1 | (0.220, −0.150, −0.025) | −135° | Ön sağ |
| T2 | (0.220, +0.150, −0.025) | +135° | Ön sol |
| T3 | (−0.220, +0.150, −0.025) | +45° | Arka sol |
| T4 | (−0.220, −0.150, −0.025) | −45° | Arka sağ |

**Dikey (Heave / Roll / Pitch):**

| Thruster | Pozisyon (x, y, z) m | Pitch açısı | Rol |
|----------|----------------------|-------------|-----|
| T5 | (0.100, −0.180, +0.035) | +90° | Ön sağ |
| T6 | (0.100, +0.180, +0.035) | +90° | Ön sol |
| T7 | (−0.100, −0.180, +0.035) | +90° | Arka sağ |
| T8 | (−0.100, +0.180, +0.035) | +90° | Arka sol |

---

## Kurulum

### Bağımlılıklar

```bash
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-gazebo-ros2-control

pip install numpy
```

### Build

```bash
cd ~/novasim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Kullanım

### Tam Simülasyon (Gazebo + RViz + Kontrol)

```bash
ros2 launch nova_bringup simulation.launch.py
```

Gazebo (sualtı ortamı) ve RViz (robot modeli + TF) açılır. Tüm kontrol node'ları otomatik başlar.

### Manuel Kontrol (Teleop)

```bash
# Ayrı bir terminalde:
ros2 run nova_control teleop
```

| Tuş | Hareket |
|-----|---------|
| `W` / `S` | İleri / Geri (Surge) |
| `A` / `D` | Sol / Sağ (Sway) |
| `Q` / `E` | Sapma sol / sağ (Yaw) |
| `Y` / `H` | Yukarı / Aşağı (Heave) |
| `T` / `G` | Hız artır / azalt |
| `Space` | Dur |

### Sadece Robot Modeli (Gazebo'suz)

```bash
ros2 launch nova_description display.launch.py
```

### Sistemi Doğrulama

```bash
# Çalışan node'lar
ros2 node list

# Thruster PWM çıkışını izle
ros2 topic echo /thrusters/commands

# Wrench komutunu izle
ros2 topic echo /control/wrench_cmd

# TF ağacını görüntüle
ros2 run tf2_tools view_frames
```

---

## Paket Yapısı

```
nova_sim_ws/
├── docs/
│   ├── gazebo_sim.png         # Gazebo sualtı simülasyon ekran görüntüsü
│   ├── rviz_robot.png         # RViz robot modeli
│   ├── rviz_tf.png            # RViz TF + robot
│   └── rviz_tf_only.png       # RViz sadece TF çerçeveleri
│
└── src/
    ├── nova_msgs/             # Özel mesaj tipleri
    │   └── msg/
    │       ├── ThrusterCommand.msg   # 8× PWM değeri (1100-1900 μs)
    │       ├── DvlData.msg           # DVL hız + geçerlilik + irtifa
    │       ├── PressureData.msg      # Basınç sensörü (derinlik m, Pa)
    │       └── ImuData.msg           # IMU (quaternion, açısal hız, ivme)
    │
    ├── nova_description/      # Robot tanımı
    │   ├── urdf/
    │   │   ├── nova.urdf.xacro       # Ana URDF (8 thruster + gövde)
    │   │   └── nova.gazebo.xacro     # Gazebo fizik plugin'leri
    │   ├── meshes/nova26.stl         # AUV gövde 3D modeli
    │   ├── worlds/underwater.world   # Gazebo sualtı dünyası
    │   ├── rviz/nova.rviz            # RViz yapılandırması
    │   └── launch/display.launch.py  # Sadece RViz
    │
    ├── nova_control/          # Kontrol algoritmaları
    │   └── nova_control/
    │       ├── thruster_allocator.py  # 6×8 TAM → PWM çözümü
    │       ├── cmd_vel_to_wrench.py   # Twist → Wrench dönüşümü
    │       ├── hydrodynamics.py       # Kuadratik sürükleme simülasyonu
    │       ├── gazebo_bridge.py       # Wrench → Gazebo plugin köprüsü
    │       ├── pid_controller.py      # Derinlik/başlık/ilerleme PID
    │       └── teleop.py              # Klavye ile 6-DoF kontrol
    │
    ├── nova_bringup/          # Launch dosyaları
    │   └── launch/
    │       ├── simulation.launch.py   # Tam sistem başlatıcı
    │       └── phase1_launch.py       # Faz 1 (otonom — geliştirme aşaması)
    │
    ├── nova_sensors/          # Sensör entegrasyonu (gelecek faz)
    └── nova_autonomy/         # Otonom kontrol (gelecek faz)
```

---

## Geliştirme Yol Haritası

| Faz | Durum | İçerik |
|-----|-------|--------|
| **Faz 0** | ✅ Tamamlandı | Fizik simülasyonu, thruster konfigürasyonu, teleop |
| **Faz 1** | 🔄 Devam ediyor | DVL + IMU + basınç sensörü entegrasyonu, PID derinlik/başlık tutma |
| **Faz 2** | 📋 Planlandı | EKF sensör füzyonu, otonom derinlik/başlık tutma |
| **Faz 3** | 📋 Planlandı | Waypoint navigasyon, görev planlayıcı, engel algılama |

---

**Geliştirici:** Ahmet Selim İlhan
