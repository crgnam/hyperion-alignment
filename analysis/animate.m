matlabrc; clc; close all;

files = dir('../../renders/*.png');
v = VideoWriter('../updates/figs/render_roll.mp4','MPEG-4');
v.FrameRate = 30;
open(v);
for ii = 1:length(files)
    fname = [files(ii).folder,'\',files(ii).name];
    img = imread(fname);
    writeVideo(v,img);
end
close(v)