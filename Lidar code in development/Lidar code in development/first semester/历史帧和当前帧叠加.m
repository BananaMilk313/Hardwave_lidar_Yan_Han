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
mergeThreshold = 0.1; % 点云合并阈值（米）
maxIterations = 30; % ICP 最大迭代次数
numFrames = 0; % 帧计数

disp("开始接收点云数据并进行3D SLAM...");

% 主循环：接收点云并构建地图
tic;
while toc < 60 % 持续运行1分钟
    % 检查是否有新消息
    if ~isempty(pointsSubscriber.LatestMessage)
        % 获取最新点云消息
        msg = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msg); % 提取XYZ点云
        
        % 点云预处理
        currentCloud = pcdownsample(pointCloud(xyzPoints), 'gridAverage', 0.1);

        % 如果是第一帧，直接初始化
        if isempty(previousCloud)
            previousCloud = currentCloud;
            globalMap = pcmerge(globalMap, currentCloud, mergeThreshold);
            continue;
        end

        % 使用ICP算法配准
        tform = pcregistericp(currentCloud, previousCloud, 'MaxIterations', maxIterations);

        % 更新当前位姿
        currentPose = rigid3d(tform.T * currentPose.T);

        % 将当前点云变换到全局坐标系
        transformedCloud = pctransform(currentCloud, currentPose);

        % 合并到全局地图
        globalMap = pcmerge(globalMap, transformedCloud, mergeThreshold);

        % 更新上一帧点云
        previousCloud = currentCloud;

        % 每10帧更新一次可视化
        if mod(numFrames, 10) == 0
            figure(1);
            pcshow(globalMap, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
            title(['构建中的3D地图 | 帧数: ', num2str(numFrames)]);
            drawnow;
        end

        % 帧计数
        numFrames = numFrames + 1;
    end
end

disp("3D SLAM完成！");
disp("最终地图已生成！");

