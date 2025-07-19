clearvars;                          % Clear all variables from memory
clear integralSMCheading ALOS3D;    % Clear persistent states in controllers
close all;                          % Close all open figure windows
clc;

addpath('gnc');
addpath('model');
addpath('lib');
addpath('estimator')

%% Scenario Value
% 1 : Fossen method  
% 2 : Proposed method

scenario = 2;
disturbance = true;

SIMremus100(scenario,disturbance);