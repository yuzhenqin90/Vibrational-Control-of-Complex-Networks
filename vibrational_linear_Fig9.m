
%% This code simulates Fig. 9 in the paper "Vibrational Control of Complex Networks"

close all
clear all


% Write here the vector tspan as tinit : dt : tfinal
tmax= 10;%----set the simulation time
tspan = [0:0.005:tmax];
n=6;
%%%%%%%%%%%-----------define golbal parameters------------------------------------------------------

%%%%%%%%%%%-----------define the natural frequencies and the initial state--------------------------
x_St=[0.0835
    0.0099
    0.1805
    0.1890
    0.0982
    0.0979];



%%%%%%%%%%%-----------solove the differential equations----------------------------------------------
options_ode = odeset('RelTol',1e-9,'AbsTol',1e-9);

[t,x] = ode45(@uncontrolled,tspan,x_St,options_ode); %% solve the ODE without control 

plot(t,x)  %% Fig. 9 (d)


[t1,x1] = ode45(@controlled,tspan,x_St,options_ode); 

% [t2,x2] = ode45(@average,tspan,x_St,options_ode);
% t1=t1(1:2:end);
% x1=x1(1:2:end,:);
figure
plot(t1,x1(:,1:6)) %,t2,x2(:,1:6)  %% Fig. 9 (d)

unctr=[t,x];
ctr=[t1,x1];
stabili_structural=[unctr,ctr];

% save(fullfile('./results', 'stabili_structural.dat'), 'stabili_structural', '-ascii');

%% ---define the differential equations

function dydt = uncontrolled(t,x)

dydt = zeros(6,1);

A=[-1 1 0 0 0 0
   1 -1 0 1 0 0
   1 0 -1 1 0 0
   0 0 1 -1 0 1
   0 0 1 0 -1 0
   0 0 0 0 1 -1];
dydt = A*x;

end


function dydt = controlled(t,x)

dydt = zeros(6,1);
ep=0.04;

a12= sqrt(2)* sin(1/ep*t)/ep;
a43 = 2* sin(1/ep*t*sqrt(2))/ep;
a53 = 2* sin(1/ep*t*sqrt(2))/ep;

A=[-1 1+a12 0 0 0 0
   1 -1 0 1 0 0
   1 0 -1 1 0 0
   0 0 1+a43 -1 0 1
   0 0 1+a53 0 -1 0
   0 0 0 0 1 -1];
dydt = A*x;

end

% function dydt = average(t,x) % average system
% 
% dydt = zeros(6,1);
% % ep=0.04;
% 
% A=[-1 0 0 0 0 0
%    1 -1 0 1 0 0
%    1 0 -1 1 0 0
%    0 0 0 -1 0 1
%    0 0 0 0 -1 0
%    0 0 0 0 1 -1];
% dydt = A*x;
% 
% end




