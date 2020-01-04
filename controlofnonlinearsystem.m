%% Control of a Nonlinear Plant with Model predictive control

%% Below code is to ensure that Simulink and Simulink Control Design are
% present as the toolboxes in MATLAB
if ~mpcchecktoolboxinstalled('simulink')
    disp('Simulink is required to run this example.')
    return
end
if ~mpcchecktoolboxinstalled('slcontrol')
    disp('Simulink Control Design is required to run this example.')
    return
end

%% To describe the model and linearize it
% After defining a non-linear plant in simulink model'mpc_nlm1', we would proceed to linearize the plant at its default operating condition
% Hence, we use linearize('model_name') command to do so and store the linearized model in
% some other variable .

linear_plant = linearize('mpc_nlm1'); 


%% Define the input and output parameters for the linearized system(model).
 % Assign names to I/O variables.
linear_plant.InputName = {'Position';'Velocity';'Torque'};  
linear_plant.OutputName = {'Joint angle';'joint velocity'};
linear_plant.InputUnit = {'m' 'm/s' 'Nm'};
linear_plant.OutputUnit = {'deg' 'm/s'};


%% Design MPC Controller
 % Create the controller object for MPC with attributes as sampling period, 
 % prediction and control horizons:
sampling_period = 0.1;                          
pred_horizon = 15;
control_horizon = 3;
 %creating the object
mpcobj = mpc(linear_plant,sampling_period,pred_horizon,control_horizon);

%% Now we impose constraints on the variables of MPC
  % Specify MV constraints.
  
mpcobj.MV = struct('Min',{-3;-2;-1},'Max',{4;2;1},'RateMin',{-1000;-1000;-1000}); 

%% Define weights on the manipulated variables and controlled variables.

mpcobj.Weights = struct('MV',[0 0 0],'MVRate',[.2 .3 .2],'OV',[1 2]);

%% Design the closed-loop MPC for the lobtained linearized model, 
  %by adding the MPC block from the Model Predictive Control toolbox
  % Then run simulation by using command sim('model_name').
  
mdl1 = 'mpc_lin1';
open_system(mdl1)    % Opens simulink model for viewing scopes and graphs
sim(mdl1);           % starts simulation


%% Modify the MPC design to track the ramp signals
% In order to track a ramp signal, we use a triple integrator as an output 
% disturbance model on both outputs.

outdistmodel = tf({1 0;0 1},{[1 0 0 0],1;1,[1 0 0 0]});
setoutdist(mpcobj,'model',outdistmodel);

%% Modify the previous model'mpc_mimonon1' by using ramp signal block 
%instead of step signal, and recreate the simulink model.
% Run simulation.

mdl2 = 'mpc_lin_ramp1';
open_system(mdl2)    % Opens simulink model for viewing scopes and graphs
sim(mdl2);           % starts simulation

%% Let us reove the constraints to see the behaviour of MPC without constraints
 % When there are no constraints actively imposed on variables of MPC, then
 % MPC controller behaves like a linear controller.
 
mpcobj.MV = [];       %removes the constraints we defined earlier and sets them to zero.

%% Reset output disturbance model to default. Since we used custom disturbances 
 % and want to move back to default model use ollowing sytax;
setoutdist(mpcobj,'integrators');   

%% After removing costraints, the MPC can be considered as Linear contrroller, 
 % so we define analogous linear controller from the new MPC object
 % 'mpcobj', which is set for default disturbances

LTI = ss(mpcobj,'r');

% The input to the linear controller LTI is the vector [ym;r], where ym
% is the vector of measured outputs, and r is the vector of output references.

%% Provide the ouput reference signals to both the controllers(MPC and Linear)
 % simultaneously to see the behaviour of controller, and graaphs of
 % generated trajectories.
 % then run simulation for the new model.

refs = [1;1];                         % output references are step signals
mdl3 = 'mpc_comp1';
open_system(mdl3)       % Opens simulink model for viewing scopes and graphs
sim(mdl3);              % starts simulation


%% Compare Simulation Results
fprintf('Compare output trajectories: ||ympc-ylin|| = %g\n',norm(ympc-ylin));
disp('The MPC controller and the linear controller produce the same closed-loop trajectories.');




