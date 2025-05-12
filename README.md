# Particle Filter Based Robot Localization

## 🧩 Gerekli Paketler

- `roscpp`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf`

Hepsi `package.xml` ve `CMakeLists.txt` dosyasına eklenmiştir.

---

##  Derleme

```bash
cd ~/robotlar_ws
catkin_make
source devel/setup.bash
```

## Çalıştırma
- Terminal 1 – ROS Master
```bash
roscore
```

- Terminal 2 – Localization Node
```bash
cd ~/robotlar_ws
source devel/setup.bash
roslaunch particle_filter_localization localization.launch
```
- Terminal 3 – Bag Kaydını Başlat
```bash
cd ~/robotlar_ws
source devel/setup.bash
rosbag play src/hw3.bag --clock
```

- Terminal 4 – RViz ile Görselleştirme
```bash
cd ~/robotlar_ws
source devel/setup.bash
rviz -d src/particle_filter_localization/localization_config.rviz
```

## Değerlendirme (Evaluation)
```bash
cd ~/robotlar_ws
source devel/setup.bash
rosrun particle_filter_localization evaluation_node

```


Çıktılar 
deneme -1 
Pozisyon Hatası Ortalaması: 40.00 m
Pozisyon Hatası Std: 19.37 m
Yaw Hatası Ortalaması: 2.54 rad
Yaw Hatası Std: 1.14 rad

deneme -2 
Pozisyon Hatası Ortalaması: 15.21 m
Pozisyon Hatası Std: 6.19 m
Yaw Hatası Ortalaması: 2.18 rad
Yaw Hatası Std: 1.53 rad



