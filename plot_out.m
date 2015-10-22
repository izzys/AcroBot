function plot_out(Sim)

t = Sim.Out.T;

theta1 =  wrapToPi( Sim.Out.X(:,1) );
dtheta1 = Sim.Out.X(:,2) ;
theta2  =  wrapToPi( Sim.Out.X(:,3) ) ;
dtheta2 =  Sim.Out.X(:,4) ;


alpha = Sim.Out.X(:,5) ;

torque = Sim.Out.Torque;
Ep = Sim.Out.Ep-Sim.Con.InitialPotentialNrg;
Ek = Sim.Out.Ek;
Etot =  Sim.Out.Etot-Sim.Con.InitialPotentialNrg;
out_time = Sim.Out.out_time;


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
plot(t,alpha,'Color','g','LineStyle','--')
ylabel('\theta_2 [rad]')
xlabel('Time')

subplot 324
plot(t,dtheta2,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('\theta_2 dot [rad/sec]')
xlabel('Time')

subplot 325
plot(out_time,torque,'Color',Color,'LineStyle',LineStyle)
hold on
ylabel('Torque [Nm]')
xlabel('Time')

subplot 326
hold on
plot(out_time,Ep,'marker','o','LineStyle',LineStyle,'Color',Color);
plot(out_time,Ek,'marker','s','LineStyle',LineStyle,'Color',Color);
plot(out_time,Etot,'marker','.','LineWidth',2,'LineStyle',LineStyle,'Color',Color);

legend('potential','kinetic','total')
ylabel('Energy [J]')
xlabel('Time')

