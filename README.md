# aimibot-ros2
## 安装依赖
```
sudo apt-get install ros-foxy-ecl-*
sudo apt install ros-foxy-nav2-*
sudo apt install ros-foxy-joint-state-publisher
```
## 安装串口功能包
```
cd [project_ws]/src
git clone https://github.com/RoverRobotics-forks/serial-ros2
cd serial-ros2
make
make install
```
## 编译
编译中途报错，原因是找不到编译好的serial，重新加载工作空间再编译
```
colcon build
source install/setup.sh
colcon build
```
