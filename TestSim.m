clc;%clear all;close all;clear classes;
set(0,'DefaultFigureWindowStyle','normal')

Sim = Simulation();
Sim.Graphics = 1;
%1: number of steps, 2: covergance
Sim.EndCond = 2;%[1;124];%[1;40];
Sim = Sim.SetTime(0,0.05,1200);

% Set up the model:
Sim.Mod = Sim.Mod.Set('m',2.8,'damping',0);

% Init controller:
Sim.Con = Sim.Con.Set('omega0',5.76439,'Gain',0,'Pulse_Width',0.25); % For pulses 
Sim.Con = Sim.Con.Set('gamma',5,'mu',0.1);% For Hopf oscillator
Sim.Con = Sim.Con.Set('NumOfNeurons',2,'epsilon',1,'tau',1,'eta',5);% For Adaptive Hopf oscillator
Sim.Con = Sim.Con.Set('Amp_teach',1,'Omega_teach',5.76439,'Phi_teach',pi/2);% Teaching signal
Sim.Con.Controller_Type = 'Hopf_adaptive';
Sim.Con.IC = [1;0;Sim.Con.omega0;1;0;];
Sim.Con.Init();

% Simulate:
Sim.Mod.IC = [-0.2 ; 0];
Sim = Sim.Init();
Sim = Sim.Run();
plot_out;