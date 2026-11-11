# nova_sensors

Bu paket NOVA AUV simülasyonuna sensör eklenmesi için ayrılmıştır.

## Eklenecek Sensörler

| Sensör | Gazebo Plugin | Topic |
|--------|--------------|-------|
| IMU (BNO085) | `libgazebo_ros_imu_sensor.so` | `/imu/data` |
| DVL A50 | custom plugin veya mock node | `/dvl/data` |
| Ping360 Sonar | `libgazebo_ros_ray_sensor.so` | `/sonar/scan` |
| Basınç Sensörü | custom mock node | `/pressure/data` |
| Derinlik Kamerası | `libgazebo_ros_camera.so` | `/camera/image_raw` |

## Nasıl Eklenir

1. URDF'e sensör link'i ekle (`nova_description/urdf/nova.urdf.xacro`)
2. Gazebo plugin'ini `nova_description/urdf/nova.gazebo.xacro` dosyasına ekle
3. Bu pakete sensör node'unu yaz (gerekirse)
4. Launch dosyasını `nova_bringup/launch/simulation.launch.py` içinde güncelle

## Bağımlılıklar

```bash
sudo apt install ros-humble-gazebo-plugins
```
