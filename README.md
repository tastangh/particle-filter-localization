# Particle Filter Based Robot Localization

## Proje Açıklaması

Bu proje, bir mobil robotun konumunu bir parçacık filtresi (particle filter) kullanarak tahmin eder. Konum tahmini, robotun `/odom` (husky_velocity_controller/odom) verileri ile güncellenir, `/front/scan` ile lidar verisiyle filtrelenir ve en iyi tahmin `/particle_pose` topiği ile yayınlanır. Ayrıca RViz ile görselleştirme ve gerçek konum ile karşılaştırmalı doğruluk analizi (`evaluation_node`) yapılır.

---

## 📁 Dosya Yapısı

robotlar_ws/
├── src/
│ ├── particle_filter/ # (Boş bırakılabilir)
│ └── particle_filter_localization/
│ ├── launch/
│ │ └── localization.launch # Başlatma dosyası
│ ├── src/
│ │ ├── particle_filter_localization_node.cpp # Algoritma
│ │ └── evaluation_node.cpp # Gerçek konumla karşılaştırma
│ ├── package.xml # ROS package dosyası
│ ├── CMakeLists.txt # CMake yapılandırması
│ └── localization_config.rviz # RViz konfigürasyonu
├── hw3.bag # Verilen bag dosyası
├── build/ # catkin_make ile oluşur
└── devel/ # catkin_make ile oluşur


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

## RViz Görselleştirme

localization_config.rviz dosyasında aşağıdakiler yer alır:

    Grid (zemin ızgarası)

    TF (map → base_link)

    Pose: /particle_pose üzerinden tahmin edilen konum (yeşil ok)

    IMU (isteğe bağlı: /imu/data eklendiğinde çizilebilir)

    Odometry: /husky_velocity_controller/odom (gerçek konum, mor ok ile)


Fixed Frame: map olmalıdır.

Eğer RViz'de No TF Data uyarısı alırsanız rosbag dosyasının --clock ile oynatıldığından emin olun.

odom verisi /husky_velocity_controller/odom, lidar ise /front/scan topiclerinden okunmaktadır.

Gelişmiş hareket modeli Denklem (1-2) ve sensör modeli Denklem (3) ile uyumludur (hw3.pdf'e göre).



Çıktılar 

Pozisyon Hatası Ortalaması: 40.00 m
Pozisyon Hatası Std: 19.37 m
Yaw Hatası Ortalaması: 2.54 rad
Yaw Hatası Std: 1.14 rad