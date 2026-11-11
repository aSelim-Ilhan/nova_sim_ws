# nova_autonomy

Bu paket NOVA AUV'un otonom navigasyon ve görev planlama kodları için ayrılmıştır.

## Planlanan Modüller

| Modül | Açıklama | Subscribes | Publishes |
|-------|----------|-----------|----------|
| `depth_controller` | Derinlik PID kontrolü | `/pressure/data`, `/imu/data` | `/cmd_vel` |
| `heading_controller` | Yaw PID kontrolü | `/imu/data` | `/cmd_vel` |
| `waypoint_navigator` | Hedef nokta navigasyonu | `/odom` | `/cmd_vel` |
| `mission_planner` | Görev sıralama | — | waypoint hedefleri |
| `ekf_node` | Konum tahmini (EKF) | `/imu/data`, `/dvl/data` | `/odom` |

## Kontrol Pipeline

```
mission_planner
    → waypoint_navigator  → /cmd_vel
    → depth_controller    → /cmd_vel
    → heading_controller  → /cmd_vel
         ↓
    cmd_vel_to_wrench (nova_control)
         ↓
    thruster_allocator (nova_control)
         ↓
    /thrusters/commands → STM32
```

## Nasıl Başlanır

1. Bu pakete yeni node ekle
2. `nova_bringup/launch/simulation.launch.py` içine node'u dahil et
3. Simülasyonda test et: `ros2 launch nova_bringup simulation.launch.py`

## Önerilen Yaklaşım

- **Faz 1:** Derinlik + yaw PID ile basit waypoint navigasyonu
- **Faz 2:** EKF ile DVL + IMU füzyonu, daha hassas konum tahmini
- **Faz 3:** Nav2 entegrasyonu, tam otonom görev
