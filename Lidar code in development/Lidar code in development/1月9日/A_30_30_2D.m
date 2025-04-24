% 清理环境
clear; clc; close all;

% 创建ROS 2节点并订阅点云话题
node = ros2node("/matlab_node"); % 创建ROS 2节点
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

% 初始化变量
slamObj = lidarSLAM(20, 8); % 地图分辨率为20栅格/米，激光雷达范围为8米
slamObj.LoopClosureThreshold = 210; % 回环检测阈值
slamObj.LoopClosureSearchRadius = 8; % 搜索半径 (米)

% 设置范围和角度限制
minRange = 0.5; % 最小范围 (米)
maxRange = 1.5; % 最大范围 (米)
minAngle = -30; % 最小角度 (度)
maxAngle = 30;  % 最大角度 (度)

previousCloud = []; % 保存上一帧点云
currentPose = rigid3d(eye(3), [0 0 0]); % 初始位姿为零
numFrames = 0; % 帧计数

disp("开始接收点云数据并进行2D SLAM...");

% 主循环：接收点云并构建地图
tic;
while toc < 60 % 持续运行1分钟
    % 检查是否有新消息
    if ~isempty(pointsSubscriber.LatestMessage)
        % 获取最新点云消息
        msg = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msg); % 提取XYZ点云

        % 提取水平切片 (z=0附近的点) 用作 2D 激光雷达数据
        zThreshold = 0.1; % 设置Z轴阈值 (米)
        horizontalPoints = xyzPoints(abs(xyzPoints(:, 3)) < zThreshold, :);

        % 计算点云的距离和角度
        distances = sqrt(horizontalPoints(:, 1).^2 + horizontalPoints(:, 2).^2);
        angles = atan2d(horizontalPoints(:, 2), horizontalPoints(:, 1)); % 转为角度 (度)

        % 过滤范围和角度
        validIndices = (distances >= minRange) & (distances <= maxRange) & ...
                       (angles >= minAngle) & (angles <= maxAngle);
        filteredPoints = horizontalPoints(validIndices, 1:2);

        % 调试：可视化点云过滤
        if mod(numFrames, 10) == 0
            figure(1);
            scatter(horizontalPoints(:, 1), horizontalPoints(:, 2), 'b.');
            hold on;
            scatter(filteredPoints(:, 1), filteredPoints(:, 2), 'r.');
            legend('原始点', '过滤后点');
            title(['点云过滤结果 | 帧数: ', num2str(numFrames)]);
            hold off;
            drawnow;
        end

        % 创建 lidarScan 对象
        if size(filteredPoints, 1) > 0
            anglesFiltered = atan2(filteredPoints(:, 2), filteredPoints(:, 1));
            rangesFiltered = sqrt(filteredPoints(:, 1).^2 + filteredPoints(:, 2).^2);
            lidarScanObj = lidarScan(rangesFiltered, anglesFiltered);

            % 如果是第一帧，直接添加
            if isempty(previousCloud)
                addScan(slamObj, lidarScanObj);
                previousCloud = lidarScanObj;
                continue;
            end

            % 添加到 Lidar SLAM 对象
            [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj, lidarScanObj);

            % 每10帧更新一次地图可视化
            if mod(numFrames, 10) == 0
                figure(2);
                show(slamObj);
                title(['构建中的2D地图 | 帧数: ', num2str(numFrames)]);
                drawnow;
            end

            % 更新上一帧点云
            previousCloud = lidarScanObj;

            % 帧计数
            numFrames = numFrames + 1;
        end
    else
        disp("未收到点云数据...");
    end
end

disp("2D SLAM完成！");
disp("最终地图已生成！");

% 构建最终地图并显示
[scans, poses] = scansAndPoses(slamObj);
finalMap = buildMap(scans, poses, slamObj.MapResolution, slamObj.MaxLidarRange);

figure;
show(finalMap);
title('最终构建的2D地图');
