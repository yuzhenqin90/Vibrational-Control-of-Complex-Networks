
close all
clear all


% Write here the vector tspan as tinit : dt : tfinal
tmax= 100;%----set the simulation time
tspan1 = [0:0.005:10];
tspan = [0:0.005:tmax];
n=4;
%%%%%%%%%%%-----------define golbal parameters------------------------------------------------------

%%%%%%%%%%%-----------define the natural frequencies and the initial state--------------------------
x_St=zeros(n,1)+0.1*rand(n,1);



%%%%%%%%%%%-----------solove the differential equations----------------------------------------------
options_ode = odeset('RelTol',1e-9,'AbsTol',1e-9);
[t,x] = ode45(@uncontrolled,tspan1,x_St,options_ode);%%% solve the ODE without control

plot(t,x)


[t1,x1] = ode45(@controlled,tspan,x_St,options_ode);  %%% solve the ODE with single input

[t2,x2] = ode45(@controlled_two_inputs,tspan,x_St,options_ode); %%% solve the ODE with two inputs


%% plot results
figure
plot(t1(1:10:end,:),x1(1:10:end,:))

figure
plot(t2(1:10:end,:),x2(1:10:end,:))

unctr=[t,x];
ctr1=[t1(1:10:end,:),x1(1:10:end,:)];
ctr2=[t2(1:10:end,:),x2(1:10:end,:)];
single_edge_unctr=unctr;
single_edge_ctr_stable=ctr1;
single_edge_ctr_unstable=ctr2;

% save(fullfile('./results/single_edge_control', 'single_edge_unctr.dat'), 'single_edge_unctr', '-ascii');
% save(fullfile('./results/single_edge_control', 'single_edge_ctr_stable.dat'), 'single_edge_ctr_stable', '-ascii');
% save(fullfile('./results/single_edge_control', 'single_edge_ctr_unstable.dat'), 'single_edge_ctr_unstable', '-ascii');


%% ---define the differential equations

function dydt = uncontrolled(t,x) %% uncontrolled

dydt = zeros(4,1);

A=[0.1  0  0  -1
    1 -1  0  0
    0  1  -.3 0
    -1  0  1  -.2];

dydt = A*x;

end


function dydt = controlled(t,x) %% single input

dydt = zeros(4,1);
ep=0.05;

a41= sqrt(16)* sin(1/ep*t)/ep;

A=[0.1  0  0  -1
    1 -1  0  0
    0  1  -.3 0
    -1+a41  0  1  -.2];

dydt = A*x;

end


function dydt = controlled_two_inputs(t,x) %% two inputs

dydt = zeros(4,1);
ep=0.05;

a21= sqrt(16)* sin(1/ep*t)/ep;
a43= -sqrt(16)* sin(1/ep*t)/ep;

A=[0.1  0  0  -1
    1+a21 -1  0  0
    0  1  -.3 0
    -1  0  1+a43  -.2];
dydt = A*x;

end
