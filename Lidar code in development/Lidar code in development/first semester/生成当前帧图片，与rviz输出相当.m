% 清理环境
clear; clc; close all;

% 创建ROS 2节点并订阅点云话题
node = ros2node("/matlab_node"); % 创建ROS 2节点
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

% 参数设置
maxIterations = 30; % ICP 最大迭代次数
numFrames = 0; % 帧计数

disp("开始接收点云数据并进行可视化...");

% 主循环：接收点云并实时显示
tic;
while toc < 60 % 持续运行1分钟
    % 检查是否有新消息
    if ~isempty(pointsSubscriber.LatestMessage)
        % 获取最新点云消息
        msg = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msg); % 提取XYZ点云
        
        % 点云预处理
        currentCloud = pcdownsample(pointCloud(xyzPoints), 'gridAverage', 0.1);

        % 可视化当前点云
        figure(1);
        cla; % 清除上一帧内容
        pcshow(currentCloud, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
        title(['当前帧点云 | 帧数: ', num2str(numFrames)]);
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        drawnow;

        % 帧计数
        numFrames = numFrames + 1;
    end
end

disp("实时点云显示完成！");

