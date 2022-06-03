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
true = outline;

N = size(limb,2);
M = size(outline,2);

% Limb Matching:
span = 1:floor(M/(360/3)):M;
% span = 1:M;
rms  = nan(1,length(span));
roll = nan(1,length(span));
kk = 1;

if animate
    v = VideoWriter('../updates/figs/alignment.mp4','MPEG-4');
    v.FrameRate = 20;
    open(v);
end

tic
for ii = span
    % Get the current hypothesis arc:
    outline_arc = get_arc(outline, ii, N);

    % Perform the alignment:
    [regParams,~,ErrorStats] = absor(outline_arc,limb, 'doScale',0);
    rms(kk) = ErrorStats.errlsq;
    R = regParams.R;
    t = regParams.t;
    
    roll(kk) = -sign(R(2,1))*acosd(R(1,1));
    kk = kk+1;
    
    % Show the current alignment:
    if animate
        a = R*outline_arc + t;
        b = R*outline + t;
        if ii == 1
            figure()
            subplot(1,2,1)
                imshow(ref); hold on
                plot(limb(1,:),limb(2,:),'r','LineWidth',3);
                plot(true(1,:),true(2,:),'--g','LineWidth',2);
                h1 = plot(b(1,:),b(2,:),'b','LineWidth',2);
                h2 = plot(a(1,:),a(2,:),'m','LineWidth',2);
                legend('Limb','Truth','Current Fit','Segment','location','northoutside')
            subplot(2,2,2)
                p1 = plot(span,rms);hold on
                xlim([0 M])
                ylim([0 200])
                grid on
                xlabel('Iteration')
                ylabel('RMS')
            subplot(2,2,4)
                p2 = plot(span,roll); hold on
                plot([0 M],[true_roll, true_roll],'g');
                xlim([0 M])
                ylim([-180 180])
                grid on
                ylabel('Estimated Roll (deg)')
                xlabel('Iteration')
        else
            set(h1,'XData',b(1,:),'YData',b(2,:))
            set(h2,'XData',a(1,:),'YData',a(2,:))
            set(p1,'YData',rms);
            set(p2,'YData',roll);
        end
        drawnow
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end
toc

if animate
    idx = find(rms == min(rms));
    roll_hat = roll(idx);
    subplot(2,2,2)
    plot(span(idx),rms(idx),'.b','MarkerSize',20)

    rotmat = euler321_to_rotmat(deg2rad(roll_hat),deg2rad(true_pitch),deg2rad(true_yaw));
    model.set_pose(zeros(3,1),rotmat);
    pixels = camera.points_to_pixels(model.vertices');
    k = boundary(pixels(1,:)',pixels(2,:)');
    outline = [pixels(1,k); pixels(2,k)];
    outline = subdivide_line(outline, subdivide, false);
    outline_arc = get_arc(outline, span(idx), N);
    [regParams,~,ErrorStats] = absor(outline_arc,limb, 'doScale',0);
    rms(ii) = ErrorStats.errlsq;
    R = regParams.R;
    t = regParams.t;


    b = R*outline + t;
    a = R*outline_arc + t;
    set(h1,'XData',b(1,:),'YData',b(2,:))
    set(h2,'XData',a(1,:),'YData',a(2,:))
    
    for jj = 1:40
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    close(v)
end