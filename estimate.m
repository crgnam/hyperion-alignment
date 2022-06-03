matlabrc; clc; close all;
addpath(genpath('src'))
addpath(genpath('lib'))
addpath(genpath('estimator'))

% Load the image to be processed:
reference_image = 'simulated_data/test_image1.png';
true_roll = 111; %(degrees)
true_pitch = 33; %(degrees)
true_yaw = 22; %(degrees)

% reference_image = 'simulated_data/test_image2.png';
% true_roll = -105;
% true_pitch = 11.3;
% true_yaw = 135;

% reference_image = 'simulated_data/test_image3.png';
% true_roll = 132;
% true_pitch = -44;
% true_yaw = 73;

% reference_image = 'simulated_data/test_image4.png';
% true_roll = 160;
% true_pitch = -45;
% true_yaw = 240;

% reference_image = 'simulated_data/test_image5.png';
% true_roll = 60;
% true_pitch = 77;
% true_yaw = 290;

%% Settings:
distance_to_hyperion = 1000;

min_particles = 100;

step_size = 10; % degrees

iterations = 5;
min_step = 1; %(degrees)
rescale = 1/2;

animate = true;

subdivide = 1;

%% Setup:
% Load in shape model:
% model = ShapeModel('data/hyperion_30k_plt.obj');
% save('data/model','model')
load('data/model')

% Define camera:
focal_length = 90/1000;
sensor_size = 36/1000;
resolution = [270,270];
cam_model = PinholeModel('FocalLength',focal_length,'SensorSize',sensor_size, 'Resolution',resolution);
camera = Camera('CameraModel',cam_model);
camera.set_pose([distance_to_hyperion;0;0], quat_to_rotmat([0.5; 0.5; 0.5; 0.5]))

%% Pre-process image:
ref = imread(reference_image);

% Obtain the limb:
limb = detect_limb(ref,1,1);

% Order limb points:
[yl,xl] = find(limb);
[xl,yl] = order_limb_points(xl,yl);
limb = [xl'; yl'];

% Get evenly spaced limb coordinates:
limb = subdivide_line(limb, subdivide, false);

%% Generate initial particle distribution:
yaw_span   = 0:deg2rad(step_size):2*pi;
pitch_span = (-pi/2):deg2rad(step_size):(pi/2);
[P,Y] = meshgrid(pitch_span,yaw_span);
pitch = P(:)';
yaw   = Y(:)';

% Randomly sample rather than use even spread:
% pitch = pi*rand(1,num_particles) - pi/2;
% yaw   = 2*pi*rand(1,num_particles);

% Particles:
particles = [yaw; pitch];
weights   = ones(size(yaw));

%% Perform Estimation:
tic
split = strsplit(reference_image,'/');
name = split{2}(1:end-4);
if animate
    v = VideoWriter(['updates/figs/distribution_',name,'.mp4'],'MPEG-4');
    v.FrameRate = 1;
    open(v);
end

figure('Position', [600 200 700 400])
for iter = 1:iterations
    fprintf('Iteration %2i/%2i (Step Size: %6.3f)\n', iter, iterations, step_size)
    yaw   = particles(1,:);
    pitch = particles(2,:);
    N = length(pitch);
    quats = zeros(4,N);
    for ii = 1:N
        rotmat = euler321_to_rotmat(0, pitch(ii), yaw(ii));
        quats(:,ii) = rotmat_to_quat(rotmat);
    end
    
    % Plot the particle distribution:
    if animate
        cla
        draw_density(particles, step_size); hold on
        plot(true_yaw,true_pitch,'ok','MarkerSize',5,'LineWidth',1)
        drawnow
        frame = getframe(gcf);
        writeVideo(v,frame);
    end

    % Run Estimation:
    rms = zeros(1,N);
    roll = zeros(1,N);
    best_rms = inf;
    for ii = 1:N
        % Update the model with current pose:
        model.set_pose(zeros(3,1), quat_to_rotmat(quats(:,ii)));

        % Generate the outline:
        pixels = camera.points_to_pixels(model.vertices');
        k = boundary(pixels(1,:)',pixels(2,:)');
        outline = [pixels(1,k); pixels(2,k)];% + [0.5; 0.5]; %(half offset since they sit in the middle of pixels)

        % Rasterize the outline (each pixel point along it):
        outline = subdivide_line(outline, subdivide, false);

        % Identify best limb match for current orientation:
        [rms(ii),R,~] = match_outline_limb(outline, limb);

        roll(ii) = -sign(R(2,1))*acos(R(1,1));

        if rms(ii) < best_rms
            best_rms = rms(ii);
            fprintf('  New Best (%3i/%3i) | yaw = %8.3f | pitch = %8.3f | roll = %8.3f | RMS: %7.3f\n',...
                     ii,N,rad2deg(yaw(ii)),rad2deg(pitch(ii)),rad2deg(roll(ii)), rms(ii))
        end
    end
    
    score = 1./rms.^3;    
    % Particle resampling:
    step_size = rescale*step_size;
    weights = ones(1,N);
    score = score/sum(score);
    color = [1-score', score', zeros(size(score))']./max(score);
    jitter = deg2rad(step_size)/3;
    weights = weights.*score;
    yaw = particles(1,:);
    pitch = particles(2,:);
    new_particles = max([min_particles, round(N*rescale)]);
    [particles,weights,index] = resample_particles(particles, weights, jitter, new_particles);
    
    % Determine best estimate using weights:
    x_hat = sum(particles.*weights,2);
end
toc
best_ind = (score == max(score));
estimated_roll  = roll(best_ind);
estimated_yaw   = yaw(best_ind);
estimated_pitch = pitch(best_ind);
fprintf('Estimate: yaw = %f, pitch %f, roll = %f\n',...
         rad2deg(estimated_yaw),rad2deg(estimated_pitch),rad2deg(estimated_roll))
if animate
    cla
    draw_density(particles, step_size); hold on
    plot(true_yaw,true_pitch,'ok','MarkerSize',5,'LineWidth',1)
    plot(rad2deg(estimated_yaw), rad2deg(estimated_pitch), 'og','MarkerSize',5,'LineWidth',2);
    plot(rad2deg(x_hat(1)), rad2deg(x_hat(2)),'om','MarkerSize',5,'LineWidth',2);
    drawnow
    frame = getframe(gcf);
    for idx = 1:3
        writeVideo(v,frame);
    end
    close(v)
end

%% Show the Estimated Orientation:
figure()
% t = tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
% nexttile
    estimated_rotmat = euler321_to_rotmat(estimated_roll, estimated_pitch, estimated_yaw);
    model.set_pose(zeros(3,1),estimated_rotmat);
    model.reset()
    model.draw('FaceColor',[.5, .5, .5], 'EdgeColor','none','SpecularStrength',0)
    light('Position',[0 1e6 0],'Style','local')
    view(90,0)
    camva(6)
    set(gca,'visible','off')
    axis equal
fname = ['updates/figs/match_',name,'.png'];
saveas(gca,fname)
% exportgraphics(t,fname,'BackgroundColor','none')
close(gcf)

% Stack with the reference image:
img = imresize(imread(fname),[nan resolution(1)]);
im_out = [ref; img];
imwrite(im_out,fname)
figure()
imshow(im_out)

%% Display the results:
Estimated = [estimated_roll; estimated_pitch; estimated_yaw];
Truth = [true_roll; true_pitch; true_yaw];
T = table( Truth,rad2deg(Estimated),'RowNames',{'Roll';'Pitch';'Yaw'});
disp(T)