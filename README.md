# UR10 Linear End-Effector Motion - HW2 Soru 2

## Proje Açıklaması

Bu ROS paketi, UR10 robot kolunun uç efektörünün iki nokta (P0 ve P1) arasında doğrusal olarak hareket etmesini sağlar. 
P0 ve P1 arasındaki çizgi, maksimum 10 cm aralıklarla bölünür ve her ara nokta için açı değerleri hesaplanarak `/ur10_arm/acilar` topiğine yayınlanır.

---

## Kurulum

### 1. Gazebo Simülasyon Ortamını Klonlayın

```bash
cd ~/robotlar_ws/src
git clone https://gitlab.com/blm6191_2425b_tai/blm6191/gazebo_plugins_rtg.git
```

### 2. Bağımlılıkları Kurun ve Ortamı Derleyin

```bash
cd ~/robotlar_ws
rosdep install -a
catkin_make
source ~/.bashrc
cp -r src/gazebo_plugins_rtg/models/ur10 ~/.gazebo/models
```

---

## Simülasyonu Başlatma

```bash
roslaunch gazebo_plugins_rtg ur10.launch
```

---

## Uç Efektörü P0'dan P1'e Gönderme

```bash
rosrun ur10_linear_motion linear_motion_node 0.4 0.2 1.0 0.9 0.2 1.0
```

Bu komut ile uç efektör `P0=(0.4, 0.2, 1.0)` noktasından `P1=(0.9, 0.2, 1.0)` noktasına hareket eder. 
Toplam 6 ara nokta oluşturularak her biri için açı hesaplanır ve robot koluna gönderilir.

---

## Eklemlere Açı Gönderme

```bash
rostopic pub /ur10_arm/acilar std_msgs/Float32MultiArray "data: [0.5, -0.2, 0.6, -0.6, -0.4, 0.5]"
```

## Pozisyon Takibi (Odometri)

```bash
rostopic echo /ur10_arm/odometri
```

---

## Simülasyonu Durdurma

```bash
rosnode kill -a
pkill -9 gzserver
pkill -9 gzclient
pkill -9 roscore
```

---

## Terminal Çıktısı

```bash
mtastan@mtastan-mtastan:~$ rosrun ur10_linear_motion linear_motion_node 0.4 0.2 1.0 0.9 0.2 1.0
[ INFO] [1744586714.801149382]: 6 ara nokta olusturuldu.
[ INFO] [1744586721.063685192, 99.223000000]: Hedefe ulasildi.
```

---

## Simülasyon Görselleri

| Açı | Görsel |
|-----|--------|
| Üstten Görünüm | ![](./1.jpg) |
| Önden Görünüm | ![](./2.jpg) |
| Yandan Görünüm | ![](./3.jpg) |
| Genel Bakış | ![](./4.jpg) |
