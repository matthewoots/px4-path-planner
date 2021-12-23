clc
clear all
close all

div = 5;
points = div * 2 + 1; % Odd to get to position 0
pi = 3.141593;
radius = 2;
height = 1.3;
s1 = linspace(0, pi, points);
s2 = linspace(-pi, 0, points);
seperation = [s1, s2(2:end)];

x = round(radius .* sin(seperation),3)';
y = round(radius .* cos(seperation),3)';
z = (ones(1,length(x)) * height)';

m = [x, y, z]; 
m_title = ["xpos", "ypos", "zpos"];
final = [m_title ; m];
writematrix(final,'wp.csv') 