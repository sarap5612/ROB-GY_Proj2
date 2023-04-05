function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    %% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    % Get K, translation_xyz from parameters.txt
    % Get the corners of all the apriltags in the data:
    res = getCorner(data(t).id);
    %Now, compute homography matrix
%     disp(res)
    A = zeros(8*length(res),9);
    n_tags = length(data(t).id);
    for i = 1:n_tags
        computed = [res(i,3) res(i,4); res(i,5) res(i,6); res(i,7) res(i,8); res(i,9) res(i,10)];
        primes = [data(t).p1(1,i) data(t).p1(2,i); data(t).p2(1,i) data(t).p2(2,i); data(t).p3(1,i) data(t).p3(2,i); data(t).p4(1,i) data(t).p4(2,i)];
        Ai = zeros(8,9);
        for j = 1:4
            Aj = [computed(j,1) computed(j,2) 1 0 0 0 -primes(j,1)*computed(j,1) -primes(j,1)*computed(j,2) -primes(j,1);
                0 0 0 computed(j,1) computed(j,2) 1 -primes(j,2)*computed(j,1) -primes(j,2)*computed(j,2) -primes(j,2)];
            Ai(((1+2*(j-1)):2*j),:) = Aj;
        end
        A(((1+8*(i-1)):8*i),:) = Ai;
    end
    [U,S,V] = svd(A);
    V_9 = V(:,9);
    V_9 = reshape(V_9,[3,3]);
    H = transpose(V_9);
    H = H*sign(V(9,9));
    t_xyz  = [-0.04, 0.0, -0.03]';
    %Now that we have the homography matrix, we can proceed to find the
    %rotation and translation using the fiven 
    K = [311.0520  0 201.8724;0 311.3885 113.6210;0 0 1];
    Rem = (K^-1)*H;
    R1 = Rem(:,1);
    R2 = Rem(:,2);
    T = Rem(:,3);
    lambda = 1/sqrt(norm((K^-1)*H(:,1))*norm((K^-1)*H(:,2)));
    R1 = lambda*R1;
    R2 = lambda*R2;
    T = lambda*T;
    [U_new,S_new,V_new] = svd(transpose([R1 R2 cross(R1,R2)]));
    R_Fin = U_new*([1 0 0; 0 1 0; 0 0 det(U_new*(V_new'))])*V_new';
    T_Fin = -R_Fin*(T);

    %% Output Parameter Defination
    R_yaw = [cos(pi/4) sin(pi/4) 0; -sin(pi/4) cos(pi/4) 0; 0 0 1];
    T_const = [R_yaw t_xyz; 0 0 0 1];
    T_overall = [R_Fin T_Fin; 0 0 0 1]*T_const;
    position = T_overall(1:3,4);
    orientation_rot = T_overall(1:3,1:3);
    [ori1,ori2] = rotm2eul(orientation_rot,'XYZ');
    orientation = ori1;
    orientation(3) = ori2(3);
    R_c2w = R_Fin;
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
end