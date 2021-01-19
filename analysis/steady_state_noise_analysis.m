height(:,1) = steadystate.VarName2/1000;
height(:,2) = steadystate.VarName3/1000;
height(:,3) = steadystate.VarName4/1000;
acc(:,1) = steadystate.VarName6/1000000;
acc(:,2) = steadystate.VarName7/1000000;
acc(:,3) = steadystate.VarName8/1000000;

% plot(acc(:,3))
% Get Variance of height

height_var = ((std(height(:,1)) + std(height(:,2)) + std(height(:,3)))/3)^2

% Get Variance of acceleration
acc_var = ((std(acc(2500:4500,1)) + std(acc(2500:4500,2)) + std(acc(2500:4500,3)))/3)^2
