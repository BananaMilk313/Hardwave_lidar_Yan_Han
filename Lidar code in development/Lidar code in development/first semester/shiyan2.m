% 清理环境
clear; clc; close all;

% 创建ROS 2节点
node = ros2node("/matlab_node"); % 创建ROS 2节点

% 订阅 /velodyne_points
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2", ...
    @pointCloudCallback); % 使用回调函数处理点云数据

disp("等待接收点云数据...");
pause(10); % 持续监听10秒

% 回调函数定义：接收并处理点云
function pointCloudCallback(msg)
    % 转换PointCloud2消息为MATLAB点云格式
    xyzPoints = rosReadXYZ(msg); % 提取XYZ坐标
    ptCloud = pointCloud(xyzPoints); % 创建MATLAB点云对象

    % 显示点云
    figure(1);
    pcshow(ptCloud, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
    title('接收到的点云数据');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

    % 简单建模：例如，平面检测
    maxDistance = 0.2; % 设置平面拟合的距离阈值
    [model, inlierIndices] = pcfitplane(ptCloud, maxDistance);

    % 可视化平面
    planeCloud = select(ptCloud, inlierIndices);
    hold on;
    pcshow(planeCloud, 'MarkerSize', 50); % 显示拟合平面
    title('拟合平面');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    drawnow;
end

