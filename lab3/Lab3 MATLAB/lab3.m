%
%% lab3 main script

simulateRR();

%% robInit testing
rob = robInit();
drawRob([0 0 0],rob);

%% creatRobTrajectory testing
rob = robInit();
via = [500 600 700 500; 0 0 0 0; 2000 1750 1250 1000];
trajectory = createRobTrajectory(via, rob);

%% robController.m testing
robot  = robInit();

via = [500 600 700 500; 0 0 0 0; 2000 1750 1250 1000];
trajectory = createRobTrajectory(via, robot);

tau = robController(trajectory, Theta, Theta_dot, t , robot )