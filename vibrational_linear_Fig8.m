%% This code simulates Fig. 8 in the paper "Vibrational Control of Complex Networks"

close all
clear all


% Write here the vector tspan as tinit : dt : tfinal
tmax=10;%----set the simulation time
tspan = [0:0.005:tmax];
tspan1 = [0:0.005:60];
n=12;
%%%%%%%%%%%-----------define golbal parameters------------------------------------------------------

%%%%%%%%%%%-----------define the natural frequencies and the initial state--------------------------

x_St=zeros(n,1)+0.2*rand(n,1);



%%%%%%%%%%%-----------solove the differential equations----------------------------------------------
options_ode = odeset('RelTol',1e-10,'AbsTol',1e-10);
[t,x] = ode45(@uncontrolled,tspan,x_St,options_ode);




[t1,x1] = ode45(@controlled,tspan1,x_St,options_ode);

% [t2,x2] = ode45(@average,tspan,x_St,options_ode);
% t1=t1(1:2:end);
% x1=x1(1:2:end,:);
figure
plot(t,x)
figure
plot(t1(1:10:end,:),x1(1:10:end,:))

general_unctr=[t(1:2:end,:),x(1:2:end,:)];
general_ctr=[t1(1:10:end,:),x1(1:10:end,:)];


% save(fullfile('./results/general', 'general_unctr.dat'), 'general_unctr', '-ascii');
% save(fullfile('./results/general', 'general_ctr.dat'), 'general_ctr', '-ascii');

%---define the differential equations

function dydt = uncontrolled(t,x)

dydt = zeros(12,1);

A = [-1, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0;
     1,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, -1, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0;
     0, 0, 1, -2, 0, 0, 0, -0.5, 0, 0, 0, 0;
     1, 0, 0, 0, -3, 1, 0, 0, 1, 0, 0, 0;
     0, 2, 0, 0, 0, -2, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, -3, 2, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0, 0, -2, 1, 1, 0;
     0, 0, 0, 0, 1, 1, 0, 0, 0, -3, 1, 0;
     0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -2, 1;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0];
dydt = A*x;

end


function dydt = controlled(t,x)

dydt = zeros(12,1);
ep=0.08;

v21 = sin(1/ep*t)/ep;
v56 = 3* sin(1/ep*t)/ep;

v43 = sqrt(3.2)*sin(sqrt(2)/ep*t)/ep;
v48 = sqrt(3)*sin(sqrt(3)/ep*t)/ep;
v78 = sqrt(15)*sin(sqrt(5)/ep*t)/ep;

v12_11 = 2*sqrt(7)*sin(sqrt(7)/ep*t)/ep;
v10_11 = sqrt(7)*sin(sqrt(7)/ep*t)/ep;
v9_11 = 0.5*sqrt(7)*sin(sqrt(7)/ep*t)/ep;



A = [-1, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0;
     1+v21,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, -1, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0;
     0, 0, 1+v43, -2, 0, 0, 0, -0.5+v48, 0, 0, 0, 0;
     1, 0, 0, 0, -3, 1+v56, 0, 0, 1, 0, 0, 0;
     0, 2, 0, 0, 0, -2, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, -3, 2+v78, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0, 0, -2, 1, 1+v9_11, 0;
     0, 0, 0, 0, 1, 1, 0, 0, 0, -3, 1+v10_11, 0;
     0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -2, 1;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1+v12_11, 0];
dydt = A*x;

end


