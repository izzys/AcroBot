clc;clear all;close all;clear classes;
set(0,'DefaultFigureWindowStyle','normal')

Sim = Simulation();
Sim.Graphics = 1;
%1: number of steps, 2: covergance
Sim.EndCond = 2;
Sim = Sim.SetTime(0,0.01,100);

% Init controller:
Sim.Con.Controller_Type = 'HC';
Sim.Con.IC = Sim.Con.alpha;
Sim.Con.Init();

% Simulate:
Sim.Mod.IC = [ 0; 0 ;0 ; 0];
Sim = Sim.Init();
Sim = Sim.Run();
plot_out(Sim);