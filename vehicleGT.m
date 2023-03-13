%
% Shows the images with a green ground truth box around the vehicle.
%
% The ground truth is estimated from the vehicle onboard sensors only.
%
clear all
close all

directory = 'REPLACE_PATH_TO_TEST'; % example: '/home/user/dataset_folder/test_1/'
vehicle = readmatrix(fullfile(directory,'vehicle.csv'));

aux = vehicle(:,1);

vehTimes = mod(aux,10000)/100;
aux = floor(aux/10000);
vehTimes = vehTimes + mod(aux,100)*60;
aux = floor(aux/100);
vehTimes = vehTimes + mod(aux,100)*3600;

vehicle(:,4) = unwrap(vehicle(:,4)*pi/180);

camParams = jsondecode(fileread(fullfile(directory,'../camera_parameters.json')));
vehParams = jsondecode(fileread(fullfile(directory,'../vehicle_parameters.json')));

camIntrinsics = cameraIntrinsics(camParams.Intrinsics.FocalLength,...
    camParams.Intrinsics.PrincipalPoint,...
    camParams.Intrinsics.ImageSize,...
    "RadialDistortion",camParams.Intrinsics.RadialDistortion,...
    "Skew",camParams.Intrinsics.Skew,...
    "TangentialDistortion",camParams.Intrinsics.TangentialDistortion);


matrixWorld2Cam = rigid3d(camParams.WorldToCam.Rotation',... % Transpose due to postmultiplication
    camParams.WorldToCam.Translation);

matrixCam2World = invert(matrixWorld2Cam);

color = [0 1 0.5];
height = 1;
len = 2;
files = dir(fullfile(directory,'img/*.png'));
fig = figure;
for i=1:length(files)
    auxTime = str2double(files(i).name(1:8));
    imgTime = mod(auxTime,10000)/100;
    auxTime = floor(auxTime/10000);
    imgTime = imgTime + mod(auxTime,100)*60;
    auxTime = floor(auxTime/100);
    imgTime = imgTime + mod(auxTime,100)*3600;
    imgTime = imgTime + 0.05; % Time offset correction
    xAntenna = interp1(vehTimes, vehicle(:,2), imgTime);
    yAntenna = interp1(vehTimes, vehicle(:,3), imgTime);
    tAntenna = interp1(vehTimes, vehicle(:,4), imgTime);


    img = imread(fullfile(files(i).folder, files(i).name));
    imshow(img);
    hold on;
    worldPoints = [0 1 1 0 0 1 1 0; 1 1 -1 -1 1 1 -1 -1; 0 0 0 0 1 1 1 1].*[vehParams.Length vehParams.Width/2 vehParams.Height]';
    worldPoints(1,:) = worldPoints(1,:) - vehParams.RearToAntenna;
    worldPoints = (eul2rotm([tAntenna 0 0])*worldPoints) + [xAntenna yAntenna, 0]';
    imgPoints = worldToImage(camIntrinsics, matrixCam2World, worldPoints', 'ApplyDistortion',true);
    hullIndex = convhull(imgPoints(:,1), imgPoints(:,2));
    plot(imgPoints(hullIndex,1), imgPoints(hullIndex,2), '-', 'Color',color,'LineWidth', 2);
    pause(0.01);
    hold off
end