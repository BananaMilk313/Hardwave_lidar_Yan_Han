% 清理环境
clear; clc; close all;

% 创建ROS 2节点并订阅点云话题
node = ros2node("/matlab_node"); % 创建ROS 2节点
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

% 初始化变量
previousCloud = []; % 保存上一帧点云
globalMap = pointCloud(zeros(0, 3)); % 初始化全局点云地图为空
currentPose = rigid3d(eye(3), [0 0 0]); % 初始位姿为零

% 参数设置
maxIterations = 50; % ICP 最大迭代次数
mergeThreshold = 0.1; % 点云合并阈值（米）
numFrames = 0; % 帧计数

disp("开始接收点云数据并进行3D SLAM...");

% 主循环：接收点云并构建地图
tic;
while toc < 60 % 持续运行1分钟
    % 获取点云消息
    msg = receive(pointsSubscriber, 10); % 超时时间为10秒
    xyzPoints = rosReadXYZ(msg); % 提取XYZ点云

    % 创建当前帧点云
    currentCloud = pointCloud(xyzPoints);

    % 如果是第一帧，直接初始化
    if isempty(previousCloud)
        previousCloud = currentCloud;
        globalMap = pcmerge(globalMap, currentCloud, mergeThreshold);
        continue;
    end

    % 使用ICP算法配准当前帧点云到上一帧点云
    tform = pcregistericp(currentCloud, previousCloud, 'MaxIterations', maxIterations);

    % 更新当前位姿
    currentPose = rigid3d(tform.T * currentPose.T); % 累积变换

    % 将当前点云变换到全局坐标系
    transformedCloud = pctransform(currentCloud, currentPose);

    % 合并到全局地图
    globalMap = pcmerge(globalMap, transformedCloud, mergeThreshold);

    % 更新上一帧点云
    previousCloud = currentCloud;

    % 可视化实时3D地图
    figure(1);
    pcshow(globalMap);
    title(['构建中的3D地图 | 帧数: ', num2str(numFrames)]);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    drawnow;

    % 帧计数
    numFrames = numFrames + 1;
end

disp("3D SLAM完成！");
disp("最终地图已生成！");

% 可选：保存地图
pcwrite(globalMap, 'global_map.pcd');
disp("地图已保存为 global_map.pcd");
