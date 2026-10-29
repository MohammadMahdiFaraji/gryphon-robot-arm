% CLICK_TO_FAST_START  Opens the window of the Gryphon robot.
% In MATLAB: open this file and press Run, or type click_to_fast_start.
% On Windows you can also double-click click_to_fast_start.bat in the same folder.
%
% MohammadMahdi Faraji, mmyasharfaraji@gmail.com

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'matlab'));
fprintf('Opening the window of the robot, this takes a few seconds...\n');
arm_gui();
