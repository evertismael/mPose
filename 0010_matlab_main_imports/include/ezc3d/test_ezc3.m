clc
clear
close all
% addpath to ezc3d here
addpath("C:\Users\evert\Documents\MATLAB\ezc3d")
% Load an empty c3d structure
c3d_to_compare = ezc3dRead('./matlab_ezc3d/d02_01.c3d');
c3d_to_compare.header.points

% Print the header
fprintf('%% ---- HEADER ---- %%\n');
fprintf('Number of points = %d\n', c3d_to_compare.header.points.size);
fprintf('Point frame rate = %1.1f\n', c3d_to_compare.header.points.frameRate);
fprintf('Index of the first point frame = %d\n', c3d_to_compare.header.points.firstFrame);
fprintf('Index of the last point frame = %d\n', c3d_to_compare.header.points.lastFrame);
fprintf('\n');
fprintf('Number of analogs = %d\n', c3d_to_compare.header.analogs.size');
fprintf('Analog frame rate = %1.1f\n', c3d_to_compare.header.analogs.frameRate);
fprintf('Index of the first analog frame = %d\n', c3d_to_compare.header.analogs.firstFrame);
fprintf('Index of the last analog frame = %d\n', c3d_to_compare.header.analogs.lastFrame);
fprintf('\n');
fprintf('\n');

% Print the parameters
fprintf('%% ---- PARAMETERS ---- %%\n');
fprintf('Number of points = %d\n', c3d_to_compare.parameters.POINT.USED.DATA);
fprintf('Name of the points =\t');
    fprintf('%s\t', c3d_to_compare.parameters.POINT.LABELS.DATA{:}); 
    fprintf('\n');
fprintf('Point frame rate = %1.1f\n', c3d_to_compare.parameters.POINT.RATE.DATA);
fprintf('Number of frames = %d\n', c3d_to_compare.parameters.POINT.FRAMES.DATA);
fprintf('Number of analogs = %d\n', c3d_to_compare.parameters.ANALOG.USED.DATA);
fprintf('Name of the analogs =\t');
    fprintf('%s\t', c3d_to_compare.parameters.ANALOG.LABELS.DATA{:}); 
    fprintf('\n');
fprintf('Analog frame rate = %1.1\n', c3d_to_compare.parameters.ANALOG.RATE.DATA);

% Print the data
fprintf('%% ---- DATA ---- %%\n');
fprintf('See figures\n');


figure('Name', '3d-Points');
h = plot3(0,0,0);
xlim([-1000, 1000]);
ylim([-1000, 1000]+1000);
zlim([-10, 2000]);
grid on; hold on;


for idx = 1:1000
    frameToPlot = idx;
    delete(h);
    h = plot3(c3d_to_compare.data.points(1,:,frameToPlot), ...
          c3d_to_compare.data.points(2,:,frameToPlot), ...
          c3d_to_compare.data.points(3,:,frameToPlot), 'k.'); 
    pause(0.01);
end
figure('Name', 'Analogs');
plot(c3d_to_compare.data.analogs);
