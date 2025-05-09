# 内容
1.Cartographer源码阅读注释
2.公司使用机器人的Cartographer参数
3.Cartographer纯定位与Nav2接口（不包括Nav2部分）

# 安装步骤
1.克隆本仓库源码
2.使用鱼香ROS一键安装的国内版rosdepc安装依赖（到~/cartographer_ws下）：
wget http://fishros.com/install -O fishros && . fishros
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
3.构建cartographer和cartographer_ros，使用--packages-up-to在构建依赖后再构建cartographer_ros：
colcon build --packages-up-to cartographer_ros
