% 清理环境
clear; clc; close all;

% 创建ROS 2节点
node = ros2node("/matlab_node"); % 创建ROS 2节点

% 订阅 /velodyne_points
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2", ...
    @(msg)disp("成功: 接收到/velodyne_points点云数据"));

% 持续监听
disp("等待接收消息...");
pause(10); % 等待10秒用于接收消息
