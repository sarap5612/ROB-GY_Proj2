function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    % First, get mapping of all of the xy coordinates
    % Get the id grid and copy-paste from parameters.txt
    april_mat_ids = [0, 12, 24, 36, 48, 60, 72, 84,  96;1, 13, 25, 37, 49, 61, 73, 85,  97;2, 14, 26, 38, 50, 62, 74, 86,  98;3, 15, 27, 39, 51, 63, 75, 87,  99;4, 16, 28, 40, 52, 64, 76, 88, 100;5, 17, 29, 41, 53, 65, 77, 89, 101;6, 18, 30, 42, 54, 66, 78, 90, 102;7, 19, 31, 43, 55, 67, 79, 91, 103;8, 20, 32, 44, 56, 68, 80, 92, 104;9, 21, 33, 45, 57, 69, 81, 93, 105;10, 22, 34, 46, 58, 70, 82, 94, 106;11, 23, 35, 47, 59, 71, 83, 95, 107];
    x1 = 0.304.*linspace(0,11,12);
    x1 = transpose(x1);
    y1 = 0.304.*linspace(0,8,9);
    y1(4:end) = y1(4:end)+0.026;
    y1(7:end) = y1(7:end)+0.026;
    y_coords = zeros(12,9);
    x_coords = zeros(12,9);
    for i = 1:9
        x_coords(:,i) = x1;
    end
    for i = 1:12
        y_coords(i,:) = y1;
    end
    %Initialize an array for res
    res = zeros(length(id),10);
    % each id will have an array length 8 which stores [p0x p0y p1x p1y p2x
    % p2y p3x p3y p4x p4y]
    % Now, we can find the corresponding coordinates for every id
    for i = 1:length(id)
        res(i,:) = zeros(1,10);
        [xi,yi] = find(april_mat_ids==id(i));
        res(i,9) = x_coords(xi,yi);
        res(i,10) = y_coords(xi,yi);
        res(i,7) = res(i,9);
        res(i,8) = res(i,10)+0.152;
        res(i,5) = res(i,7)+0.152;
        res(i,6) = res(i,8);
        res(i,4) = res(i,10);
        res(i,3) = res(i,9)+0.152;
        res(i,2) = res(i,10)+(0.152/2);
        res(i,1) = res(i,9)+(0.152/2);
    end
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
end