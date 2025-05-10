roscore


cd ~/robotlar_ws
source devel/setup.bash
roslaunch particle_filter_localization localization.launch


cd ~/robotlar_ws/src
rosbag play hw3.bag


rviz -d ~/robotlar_ws/localization_config.rviz


rosnode list


📁 robotlar_ws/
├── 📁 src/
│   ├── 📁 particle_filter/                        # (Ödev gereği boş bırakılabilir veya temel yapı varsa eklenir)
│   └── 📁 particle_filter_localization/
│       ├── 📁 launch/
│       │   └── localization.launch                # Launch dosyası
│       ├── 📁 src/
│       │   └── particle_filter_localization_node.cpp  # Ana kod dosyan
│       ├── package.xml                           # Paket tanımı
│       └── CMakeLists.txt                        # Derleme betiği
├── build/                                        # catkin_make tarafından oluşturulur
├── devel/                                        # catkin_make tarafından oluşturulur
├── hw3.bag                                       # Verilen bag dosyası
└── localization_config.rviz                      # RViz konfigürasyon dosyası


// 📄 localization.launch
<launch>
    <param name="use_sim_time" value="true"/>
    <node pkg="particle_filter_localization" 
          type="particle_filter_localization_node" 
          name="particle_filter_node" 
          output="screen"/>
</launch>


// 📄 CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(particle_filter_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(particle_filter_localization_node src/particle_filter_localization_node.cpp)

target_link_libraries(particle_filter_localization_node
  ${catkin_LIBRARIES}
)


// 📄 package.xml
<?xml version="1.0"?>
<package format="2">
  <name>particle_filter_localization</name>
  <version>0.0.0</version>
  <description>Particle filter based localization</description>

  <maintainer email="you@example.com">your_name</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf</depend>

  <export></export>
</package>


// ✅ Başlatmak için terminal adımları:
# Terminal 1:
roscore

# Terminal 2:
cd ~/robotlar_ws && source devel/setup.bash
roslaunch particle_filter_localization localization.launch

# Terminal 3:
cd ~/robotlar_ws && source devel/setup.bash
rosbag play hw3.bag --clock

# Terminal 4:
rviz -d ~/robotlar_ws/localization_config.rviz
