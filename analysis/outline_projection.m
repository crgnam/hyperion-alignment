% matlabrc; clc; close all;
addpath(genpath('../src'))
addpath(genpath('../lib'))
addpath(genpath('../estimator'))

% Load in the image:
reference_image = '../simulated_data/test_image5.png';
true_roll = 60;
true_pitch = 77;
true_yaw = 290;

subdivide = 0.5;

animate = true;

%% Setup:
% Load in shape model:
% model = ShapeModel('../data/hyperion_30k_plt.obj');
% save('../data/model','model')
load('../data/model')

distance_to_hyperion = 1000;

% Define camera:
focal_length = 90/1000;
sensor_size = 36/1000;
resolution = [270,270];
cam_model = PinholeModel('FocalLength',focal_length,'SensorSize',sensor_size, 'Resolution',resolution);
camera = Camera('CameraModel',cam_model);
camera.set_pose([distance_to_hyperion;0;0], quat_to_rotmat([0.5; 0.5; 0.5; 0.5]))

%% Analysis:
% Pre-process the image:
ref = imread(reference_image);
limb = detect_limb(ref,1,1);
[yl,xl] = find(limb);
[xl,yl] = order_limb_points(xl,yl);
limb = [xl'; yl'];
limb = subdivide_line(limb, subdivide, false);

% Set the true pose of the body:
rotmat = euler321_to_rotmat(deg2rad(true_roll),deg2rad(true_pitch),deg2rad(true_yaw));
model.set_pose(zeros(3,1),rotmat);
pixels = camera.points_to_pixels(model.vertices');
k = boundary(pixels(1,:)',pixels(2,:)');
outline = [pixels(1,k); pixels(2,k)];
outline = subdivide_line(outline, subdivide, false);

plot(pixels(1,:),pixels(2,:),'.k','MarkerSize',2); hold on
plot(outline(1,:),outline(2,:),'b','LineWidth',2)
axis equal
xlim([0 resolution(1)])
ylim([0 resolution(2)])
set(gca,'YDir','reverse')

% imshow(ref); hold on
% plot(limb(1,:),limb(2,:),'r','LineWidth',2)