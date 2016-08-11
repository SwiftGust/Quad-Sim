%{
 This demonstration script generates a timeseries in the workspace
 After saving the workspace, you can load it later to a position controler 
 as a timeseries of position commands.

Additional quadcopter modeling and simulation materials available at:
github.com/dch33/Quad-Sim

Copyright (C) 2014 D. Hartman, K. Landis, S. Moreno, J. Kim, M. Mehrer

License:
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

%% This demonstration script generates a timeseries in the workspace
%
% X = 100*[0 1 1 0 0]; % meters
% Y = 100*[0 0 1 1 0]; % meters
% Z = 100*[1 1 1 1 1]; % meters
% t = 20*[0 1 2 3 4]; % seconds
% Psi = [0 90 180 270 0]*pi/180; % radians
% path.x = timeseries(X,t);
% path.y = timeseries(Y,t);
% path.z = timeseries(Z,t);
% path.psi = timeseries(Psi,t);
% clear X Y Z t Psi
% uisave('path', 'Path_Large')
%clear path
%% % This demonstration script generates a much more absurd path.
% % Note that it takes a long time to run and each step is five seconds
% % Team 37 Plot
% X = [0,0,-1,1,4,   1,  4,   1,4,5.5,6.25,4.75,6.25,7,7.25,8.5,9.25,10,10,12,  14, 12,  14,12,15,17, 16,15,0];% meters 28pts
% X = X*0.3048; meter to feet conversion
% Y = [0,3, 3,3,3,2.25,1.5,0.75,0,  3, 1.5, 1.5, 1.5,0,   3,   0,  3, 0, 0, 0,0.75,1.5,2.25, 3, 3, 3,1.5, 0,0];
% Y = Y*0.3048; meter to feet conversion
% Z = [3];
% 
%% Diamond Path;
%  X = 100 * [0 1 2 1 0 1];
%  Y = 100 * [0 0 1 2 1 0]; 
%  Z = 100;

%% Rectangle Path;
% X = 100 * [0 1 1 0 0];
% Y = 100 * [0 0 1 1 0]; 
% Z = 100;
% tXY = 1:10:10*length(X); % seconds

%% Helix Path
tXY = 0:0.1:100; % seconds
X = 0.5.*exp(0.1*tXY).*cos(tXY); 
Y = 0.5.*exp(0.1*tXY).*sin(tXY);
Z = 1 + 0.5 * tXY;
% 
Psi = [0]; % radians
path.x = timeseries(X,tXY);
path.y = timeseries(Y,tXY);
path.z = timeseries(Z,tXY);
path.psi = timeseries(Psi,tXY);
clear X Y Z tXY Psi 
uisave('path', 'Path_Exponential_Helix')
%clear path