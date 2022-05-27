matlabrc; clc; close all;
addpath(genpath('src'))
addpath(genpath('lib'))
addpath(genpath('estimator'))

% Load the image to be processed:
reference_image = 'simulated_data/test_image0.png';
true_roll = 111; %(degrees)
true_pitch = 33; %(degrees)
true_yaw = 22; %(degrees)

% reference_image = 'simulated_data/test_image1.png';
% true_roll = -105;
% true_pitch = 11.3;
% true_yaw = 135;


%% Settings:
distance_to_hyperion = 1000;

min_particles = 50;

step_size = 10; % degrees

iterations = 5;
min_step = 1; %(degrees)
rescale = 1/2;

animate = true;

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
limb = subdivide_line(limb, 1, false);

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
if animate
    v = VideoWriter('distribution.mp4','MPEG-4');
    v.FrameRate = 1;
    open(v);
end

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
        plot(true_yaw,true_pitch,'xk','MarkerSize',5,'LineWidth',1)
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
        outline = subdivide_line(outline, 1, false);

        % Identify best limb match for current orientation:
        [rms(ii),R,~] = match_outline_limb(outline, limb);

        roll(ii) = -sign(R(2,1))*acos(R(1,1));

        if rms(ii) < best_rms
            best_rms = rms(ii);
            fprintf('  New Best (%3i/%3i) | yaw = %8.3f | pitch = %8.3f | roll = %8.3f | RMS: %7.3f\n',...
                     ii,N,rad2deg(yaw(ii)),rad2deg(pitch(ii)),rad2deg(roll(ii)), rms(ii))
        end
    end
    
    % Do not resample if max iterations has been reached:
    score = 1./rms.^3;
    if iter == iterations
        break
    end
    if step_size < min_step
        break
    end
    
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
end
toc
best_ind = (score == max(score));
estimated_roll  = roll(best_ind);
estimated_yaw   = yaw(best_ind);
estimated_pitch = pitch(best_ind);
fprintf('Estimate: yaw = %f, pitch %f, roll = %f\n',...
         rad2deg(estimated_yaw),rad2deg(estimated_pitch),rad2deg(estimated_roll))
if animate
%     plot(rad2deg(estimated_yaw), rad2deg(estimated_pitch), 'og','MarkerSize',5,'LineWidth',2);
    close(v)
end

%% Show the Estimated Orientation:
figure()
subplot(1,2,1)
    imshow(ref)
    title('Reference Image')

subplot(1,2,2)
    estimated_rotmat = euler321_to_rotmat(estimated_roll, estimated_pitch, estimated_yaw);
    model.set_pose(zeros(3,1),estimated_rotmat);
    model.reset()
    model.draw('FaceColor',[.5, .5, .5], 'EdgeColor',[.3,.3,.3])
    view(90,0)
    xlim([-inf inf]); ylim([-inf inf]); zlim([-inf inf])
    title('Estimated Orientation')
    set(gca,'XTick',[]); set(gca,'YTick',[]); set(gca,'ZTick',[]);
    set(gca,'XTickLabel',[]); set(gca,'YTickLabel',[]); set(gca,'ZTickLabel',[]);
set(findall(gcf,'-property','FontSize'),'FontSize',20)