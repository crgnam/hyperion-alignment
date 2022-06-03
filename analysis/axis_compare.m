matlabrc; clc; close all;
addpath(genpath('../src'))
addpath(genpath('../lib'))
addpath(genpath('../estimator'))

%% Setup:
% Load in shape model:
% model = ShapeModel('../data/hyperion_30k_plt.obj');
% save('../data/model','model')
load('../data/model')

figure('Position', [600 200 800 800])
model.draw('FaceColor',[.5 .5 .5],'EdgeColor',[.3 .3 .3])
axis equal
v = VideoWriter('../updates/figs/yaw.mp4','MPEG-4');
v.FrameRate = 30;
open(v);
for ii = linspace(0,360,10*30)
    rotmat = euler321_to_rotmat(0,0,deg2rad(ii));
    model.set_pose(zeros(3,1),rotmat);
    model.draw()
    xlim([-2000 2000])
    ylim([-2000 2000])
    zlim([-2000 2000])
    camva(0.5)
    view(90,0)
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v)

v = VideoWriter('../updates/figs/pitch.mp4','MPEG-4');
v.FrameRate = 30;
open(v);
for ii = linspace(0,360,10*30)
    rotmat = euler321_to_rotmat(0,deg2rad(ii),0);
    model.set_pose(zeros(3,1),rotmat);
    model.draw()
    xlim([-2000 2000])
    ylim([-2000 2000])
    zlim([-2000 2000])
    camva(0.5)
    view(90,0)
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v)

v = VideoWriter('../updates/figs/roll.mp4','MPEG-4');
v.FrameRate = 30;
open(v);
for ii = linspace(0,360,10*30)
    rotmat = euler321_to_rotmat(deg2rad(ii),0,0);
    model.set_pose(zeros(3,1),rotmat);
    model.draw()
    xlim([-2000 2000])
    ylim([-2000 2000])
    zlim([-2000 2000])
    camva(0.5)
    view(90,0)
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v)