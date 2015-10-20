function plot_out(Sim)

theta1 = Sim.Out.X(:,1);
dtheta1 = Sim.Out.X(:,2);
theta2 = Sim.Out.X(:,3);
dtheta2 = Sim.Out.X(:,4);


torque = Sim.Out.Torque;
torque_time = Sim.Out.Torque_time;
t = Sim.Out.T;



figure(121)

subplot 221
plot(t,theta1)
ylabel('\theta_1 [rad]')
xlabel('Time')

subplot 222
plot(t,dtheta1)
ylabel('\theta_1 dot [rad/sec]')
xlabel('Time')

subplot 223
plot(t,theta2)
ylabel('\theta_2 [rad]')
xlabel('Time')

subplot 224
plot(t,dtheta2)
ylabel('\theta_2 dot [rad/sec]')
xlabel('Time')