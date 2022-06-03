% matlabrc; clc; close all;
addpath(genpath('../src'))
addpath(genpath('../lib'))
addpath(genpath('../estimator'))

%% Setup:
% Load in shape model:
% model = ShapeModel('../data/hyperion_30k_plt.obj');
% save('../data/model','model')
load('../data/model')

figure('Position', [600 200 1000 1000])
model.draw('FaceColor',[.5 .5 .5],'EdgeColor',[.3 .3 .3])
axis equal
xlim([-1000 1000])
ylim([-1000 1000])
zlim([-1000 1000])
camva(1)
view([0 10])

v = VideoWriter('../updates/figs/model.mp4','MPEG-4');
v.FrameRate = 30;
open(v);
for ii = linspace(0,360,10*30)
    view([ii 10])
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v)
