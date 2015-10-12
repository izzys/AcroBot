% plot out:

theta = Sim.Out.X(:,1);
dtheta = Sim.Out.X(:,2);
phase1 = Sim.Out.X(:,3);
phase2 = Sim.Out.X(:,4);
omega = Sim.Out.X(:,5);
alpha = Sim.Out.X(:,6);
phi = Sim.Out.X(:,7);

torque = Sim.Out.Torque;
torque_time = Sim.Out.Torque_time;
t = Sim.Out.T;



figure(121)

subplot 311
plot(t,theta)
ylabel('\theta')
subplot 312
plot(t,dtheta)
ylabel('\theta dot')

subplot 313
hold on
plot(t,phase1,'--','color',[0.8 0.5 0.7],'LineWidth',0.7)
plot(t,phase2,'--','color',[0.6 0.6 0.6],'LineWidth',0.7)
stairs(torque_time,torque,'LineWidth',2)
legend('x_h','y_h')
ylabel('Torque [Nm]')
xlabel('Time [sec]')

figure(122)

%axis([-0.2 0.2  -1 1])
hold on
title('Phase plane')
plot(theta,dtheta)
plot(phase1,phase2,'--','color',[0.6 0.6 0.6],'LineWidth',0.7)
legend('Model','Ocsillator')
xlabel('\theta')
ylabel ('\theta dot')
plot(theta(1),dtheta(1),'o','MarkerSize',5)
plot(phase1(1),phase2(1),'o','MarkerSize',5,'color',[0.6 0.6 0.6])


% no reflex:
if strcmp(Sim.Con.Controller_Type, 'reflex')

    m = Sim.Mod.m;
    g = Sim.Mod.g; 
    l = Sim.Mod.L_cg; 
    I = Sim.Mod.I; 
    T = 0;%Sim.Con.Gain;
    
    
    lambda = sqrt(m*g*l/I);
    tau = T/I;
    alpha = Sim.Mod.SwitchAngle-tau/lambda^2;

    
    aa = alpha;
    bb = lambda*alpha;
    xx0 = tau/(lambda^2*alpha);
     
    tt = 0:0.001:2*pi;
    xx = aa*cos(tt)+xx0;
    yy = bb*sin(tt);
    
    figure(122)
    plot(xx,yy,'g')
    line([-0.1 -0.1],[-1 1],'LineStyle','--','color',[0.6 0.6 0.6],'LineWidth',0.7)
end




if ~isempty(Sim.Out.PoincareSection)  
    discrete_theta = Sim.Out.PoincareSection(1,:);
    discrete_dtheta = Sim.Out.PoincareSection(2,:);
    
    figure(122)
    plot(discrete_theta,discrete_dtheta,'rx','MarkerSize',5)
    
    figure(123)
    hold on
    title('Poincare section')
    % subplot 211
    % plot(discrete_theta,'rx','MarkerSize',8)
    % ylabel('\theta')
    % 
    % subplot 212
    plot(discrete_dtheta,'rx','MarkerSize',8,'LineWidth',2)
    ylabel ('\theta dot')
    xlabel ('# of steps')
end

figure
Pteach = Sim.Con.Amp_teach*sin(Sim.Con.Omega_teach*t+Sim.Con.Phi_teach);
Qlearn = alpha.*phase1;
plot(t,Pteach,t,Qlearn)
legend('Pteach','Qlearn')
xlabel('Time [sec]')

figure

for i=0:Sim.Con.NumOfNeurons-1
omega(:,i+1) = Sim.Out.X(:,5+i*5);
alpha(:,i+1) = Sim.Out.X(:,6+i*5);
phi(:,i+1) = Sim.Out.X(:,7+i*5);

plot(t,alpha(:,i+1),t,omega(:,i+1),t,phi(:,i+1))
end

plot(t,alpha,t,omega,t,phi)
legend('\alpha','\omega','\phi')
xlabel('Time [sec]')
