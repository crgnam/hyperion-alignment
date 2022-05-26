matlabrc; clc; close all;
addpath(genpath('src'))
addpath(genpath('lib'))
addpath(genpath('estimator'))

% Settings:
distance_to_hyperion = 1000;
reference_image = 'simulated_data/reference_3.png';
max_iterations = 10;

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
limb = detect_limb(ref,2,1);

% Order limb points:
[yl,xl] = find(limb);
[xl,yl] = order_limb_points(xl,yl);
limb = [xl'; yl'];

% Get evenly spaced limb coordinates:
limb = subdivide_line(limb, 1, false);

%% Generate initial particle distribution:
num_yaw = 36/2;
num_pitch = 18/2;
yaw_span   = linspace(0,2*pi,num_yaw);
pitch_span = linspace(-pi/2, pi/2,num_pitch);

N = num_yaw*num_pitch;
quats = zeros(4,N);
[P,Y] = meshgrid(pitch_span,yaw_span);
pitch = P(:)';
yaw   = Y(:)';
weights = ones(length(pitch),1);

%% Perform Estimation:
for iter = 1:max_iterations
    fprintf('Beginning Iteration %i/%i\n', iter, max_iterations)
    for ii = 1:N
        rotmat = euler321_to_rotmat(0, pitch(ii), yaw(ii));
        quats(:,ii) = rotmat_to_quat(rotmat);
    end
    
    
    % Show the current distribution:
%     cla
%     draw_density(rad2deg(yaw), rad2deg(pitch),...
%                  linspace(0,360,round(360/num_pitch)),...
%                  linspace(-90,90,round(180/num_yaw))); hold on
%     drawnow
    if iter == 1
        h1 = plot(rad2deg(yaw),rad2deg(pitch),'.k'); hold on
    else
        set(h1,'XData',rad2deg(yaw), 'YData',rad2deg(pitch)) 
    end

    % Run Estimation:;
    rms = zeros(N,1);
    score = zeros(N,1);
    roll = zeros(N,1);
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

        roll(ii) = -sign(R(2,1))*acosd(R(1,1));
        score(ii) = 1/rms(ii);

        if rms(ii) < best_rms
            best_rms = rms(ii);
%             fprintf('    New Best RMS: %f (found in particle: %i/%i)\n', rms(ii), ii, N);
        end
        fprintf('%i/%i\n',ii,N)
    end
    
    % Resample particles:
    new_weights = weights.*score;
    rescaled_weights = new_weights/sum(new_weights);
    jitter = deg2rad(2);
    
    [particles,weights] = resample_particles([pitch; yaw], rescaled_weights, jitter);
    pitch = particles(1,:);
    yaw = particles(2,:);
end

% Show the current distribution:
cla
draw_density(rad2deg(yaw), rad2deg(pitch),...
             linspace(0,360,round(360/num_pitch)),...
             linspace(-90,90,round(180/num_yaw)));
%         plot(rad2deg(yaw),rad2deg(pitch),'.k')
drawnow

%% Plot Results:
best_ind = score==max(score);
estimated_roll = deg2rad(roll(best_ind));
estimated_pitch = pitch(best_ind);
estimated_yaw = yaw(best_ind);
estimated_rotmat = euler321_to_rotmat(estimated_roll, estimated_pitch, estimated_yaw);

figure()
    surf(rad2deg(Y),rad2deg(P),reshape(score,size(P)),'EdgeColor','none','FaceColor','interp'); hold on
    plot3(22,33,2,'xk','MarkerSize',10,'LineWidth',3)
    plot3(rad2deg(estimated_yaw),rad2deg(estimated_pitch),2,'or','MarkerSize',10,'LineWidth',2);
    plot3(rad2deg(Y),rad2deg(P),2*ones(size(P)),'.k')
    view([0 90])
    axis equal
    xlabel('Yaw')
    ylabel('Pitch')
    xlim([0 360])
    ylim([-90 90])
    title('Matching Score (1/RMS)')
    colorbar
    legend('Similarity','Truth','Estimate')
    set(findall(gcf,'-property','FontSize'),'FontSize',20)

figure()
    subplot(1,2,1)
        imshow(ref)
        title('Reference Image')

    subplot(1,2,2)
        model.set_pose(zeros(3,1),estimated_rotmat);
        model.reset()
        model.draw('FaceColor',[.5, .5, .5], 'EdgeColor',[.3,.3,.3])
        view(90,0)
        xlim([-inf inf]); ylim([-inf inf]); zlim([-inf inf])
        title('Estimated Orientation')
        set(gca,'XTick',[]); set(gca,'YTick',[]); set(gca,'ZTick',[]);
        set(gca,'XTickLabel',[]); set(gca,'YTickLabel',[]); set(gca,'ZTickLabel',[]);
    set(findall(gcf,'-property','FontSize'),'FontSize',20)