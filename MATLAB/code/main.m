% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;

% set(gcf, 'Renderer', 'painters');
% set(gcf, 'Position', [500, 50, 700, 700]);

% Environment map in 2D space 
start_pt = [1, 1];
target_pt = [9, 9];
SIZE_X = 10;
SIZE_Y = 10;
map = obstacle_map(start_pt, target_pt, SIZE_X, SIZE_Y);

% Waypoint Generator Using the A* 
path = A_star_search(map, start_pt, target_pt);


% visualize the 2D grid map
visualize_map(map, path);
