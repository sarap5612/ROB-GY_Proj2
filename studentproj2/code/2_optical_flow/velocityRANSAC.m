function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter 
    % For each point, compute the H matrix
    As = zeros(2*length(optPos),3);
    Bs = zeros(2*length(optPos),3);
    Vel = [];
    Velint = [];
    % Need to define this with a skew-symmetric of body to IMU stuff
    c2w = blkdiag(R_c2w,R_c2w);
    %Use e (the RANSAC parameter) as a boolean, set to false to disable use
    %of RANSAC since we are creating our own RANSAC implementation.
    for i=1:length(optPos)
        zi = 1/Z(i);
        Ai = [-1 0 optPos(i,1); 0 -1 optPos(i,2)];
        Bi = [(optPos(i,1)*optPos(i,2)) -(1+(optPos(i,1))^2) optPos(i,2); (1+(optPos(i,2))^2) -optPos(i,1)*optPos(i,2) -optPos(i,1)];
        Hi = horzcat((zi.*Ai),Bi);
        H_cross = pinv(Hi);
        Veli = H_cross*(transpose(optV(i,:)));
        Veli = c2w*Veli;
        Velint = [Velint,Veli];
        Vel = [Vel;Veli];
    end

    %% Now that we have the Velocity, we can implement RANSAC:
    M = 3;
    epsilon = 0.8;
    k = (log(1-e))/(log(1-epsilon^M));
    % Select M points
    selectedPoints = zeros(M,6);
    for j = 1:k
        % Select M points randomly
        selectedPoints = zeros(M,6);
        indices = [randi(length(Velint)), randi(length(Velint))];
        selectedPoints = [Velint(indices(1),:), Velint(indices(2),:)];
        line_vel_fit = polyfit(selectedPoints(1,1:3),selectedPoints(2,1:3),1);
        ang_vel_fit = poltfit(selectedPoints(1,4:6), selectedPoints(2,4:6),1);

    end

    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
end