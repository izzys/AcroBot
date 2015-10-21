function plot_out(Sim)

t = Sim.Out.T;

theta1 =  Sim.Out.X(:,1)*180/pi ;
dtheta1 = Sim.Out.X(:,2)*180/pi ;
theta2  =   Sim.Out.X(:,3)*180/pi ;
dtheta2 =  Sim.Out.X(:,4)*180/pi ;


torque = Sim.Out.Torque;
torque_time = Sim.Out.Torque_time;


Color = [rand(1) rand(1) rand(1)];
LineStyle = '-';

figure(121)

subplot 321
plot(t,theta1,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('\theta_1 [rad]')
xlabel('Time')

subplot 322
plot(t,dtheta1,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('\theta_1 dot [rad/sec]')
xlabel('Time')

subplot 323
plot(t,theta2,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('\theta_2 [rad]')
xlabel('Time')

subplot 324
plot(t,dtheta2,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('\theta_2 dot [rad/sec]')
xlabel('Time')

subplot 325
plot(torque_time,torque,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('Torque [Nm]')
xlabel('Time')


tip_height = Sim.Mod.GetPos(Sim.Out.X','end2');


subplot 325
plot(t,tip_height,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('Torque [Nm]')
xlabel('Time')

