%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
K = [311.0520  0 201.8724;0 311.3885 113.6210;0 0 1];
t_xyz  = [-0.04, 0.0, -0.03]';

for n = 2:length(sampledData)
    %% Initalize Loop load images
    im1 = sampledData(n-1).img;
    im2 = sampledData(n).img;
    dt = sampledData(n).t - sampledData(n-1).t;
%     dt = 0.15;
    %% Detect good points
    % Use the DetectFASTfeatures
    image_corners = detectFASTFeatures(im1);
    pre_img = image_corners.selectStrongest(25);
    %% Initalize the tracker to the last frame.
    tracker = vision.PointTracker('MaxBidirectionalError',1);
    initialize(tracker,pre_img.Location,im1)
    %% Find the location of the next points;
    optPos = step(tracker,im2);
    %% Calculate velocity
    % Use a for loop
    %Augment the optPos and pre_img_points matrices
    optPos(:,3) = ones(length(optPos),1);
    pre_img_points = pre_img.Location;
    pre_img_points(:,3) = ones(length(pre_img_points),1);
    %Normalize optPos
    optPos = (inv(K))*optPos';
    optPos = optPos';
    % Normalize pre_img_points
    pre_img_pts = (inv(K))*pre_img_points';
    pre_img_pts = pre_img_pts';
    optV = zeros(length(optPos),2);
%     Z = zeros(length(optPos),1);
    for i = 1:length(optPos)
        u = (optPos(i,1) - pre_img_pts(i,1))*(1/dt);
        v = (optPos(i,2) - pre_img_pts(i,2))*(1/dt);
        optV(i,:) = [u,v];
    end

    %% Calculate Height
    [position, orientation, R_c2w] = estimatePose(sampledData,n);
%     position = R_c2w*position;
    % Use normalized image coordinates, and you get the world in the camera
    % frame
    Z = (-R_c2w(3,1)*optPos(:,1) - R_c2w(3,2)*optPos(:,2) - position(3))/R_c2w(3,3);
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    e = 1;
    [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e);
    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    % sgolayfilt somehow(?)
%     linear_cam_vel = Vel(1:3,:);
%     angular_cam_vel = Vel(4:6,:);
%     Vel_world = [R_c2w'*linear_cam_vel;R_c2w*angular_cam_vel];
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
%     estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
