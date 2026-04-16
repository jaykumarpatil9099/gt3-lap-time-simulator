%% startup_project.m
%  Run this script once when you open MATLAB to work on the N24 lap sim.
%  It sets the current directory to the project root and loads the car
%  parameters into the workspace so everything is ready to go.
%
%  Usage:
%    >> cd 'C:\Users\jayku\Documents\Claude\Projects\Lap time simulator'
%    >> startup_project
%
%  Or just double-click this file in MATLAB's file browser.

%% Set working directory to this script's location (= project root)
project_root = fileparts(mfilename('fullpath'));
cd(project_root);
fprintf('Project root set to: %s\n', project_root);

%% Add project subfolders to the MATLAB path (this session only, not saved)
%  This lets MATLAB find our scripts without hard-coded paths.
%  addpath with -end puts our folders at the BOTTOM of the path so they
%  don't override anything in your default MATLAB installation.
addpath(genpath('02_data'), '-end');
addpath(genpath('03_models'), '-end');
addpath(genpath('04_correlation'), '-end');
addpath(genpath('05_studies'), '-end');
fprintf('Project folders added to path (session only).\n');

%% Load vehicle parameters
run('02_data\car\amg_gt3_params.m');
fprintf('Car parameters loaded: %s\n\n', car.meta.name);
