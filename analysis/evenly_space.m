matlabrc; clc; close all;% matlabrc; clc; close all;
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
limb_i = [xl'; yl'];
limb = subdivide_line(limb_i, subdivide, false);

% Set the true pose of the body:
rotmat = euler321_to_rotmat(deg2rad(true_roll),deg2rad(true_pitch),deg2rad(true_yaw));
model.set_pose(zeros(3,1),rotmat);
pixels = camera.points_to_pixels(model.vertices');
k = boundary(pixels(1,:)',pixels(2,:)');
outline_i = [pixels(1,k); pixels(2,k)];
outline = subdivide_line(outline_i, subdivide, false);

subplot(1,2,1)
    plot(limb_i(1,:), limb_i(2,:),'.k'); hold on
    plot(limb(1,:), limb(2,:),'og');
    axis equal
    grid on
subplot(1,2,2)
    plot(outline_i(1,:), outline_i(2,:), '.k'); hold on
    plot(outline(1,:),outline(2,:),'og')
    axis equal
    grid on