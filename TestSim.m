clc;%clear all;close all;clear classes;
set(0,'DefaultFigureWindowStyle','normal')

Sim = Simulation();
Sim.Graphics = 0;
%1: number of steps, 2: covergance
Sim.EndCond = 2;%[1;124];%[1;40];
Sim = Sim.SetTime(0,0.05,1200);

% Init controller:
Sim.Con.Init();

% Simulate:
Sim.Mod.IC = [-0.2 ; 0 ; 0 ; 0];
Sim = Sim.Init();
Sim = Sim.Run();
plot_out;