function [pos vel Px] = DvKalman(z, dt)
% 
% 
persistent A H Q R 
persistent x P
persistent firstRun


if isempty(firstRun)
  firstRun = 1;
  
  H = [1 0]; 
  R = 10;       % measurement noise variance
  x = [ 0 0 ]';
  P = 5*eye(2); % uncertainty matrix, initial guess (will get corrected)
end


A = [ 1 dt ;
        0 1  ]; 

Q = [ (dt^3)/3, (dt^2)/2; 
        (dt^2)/2,    dt   ]; % process noise covariance

% kalman filter algorithm

xp = A*x;  % xp is x_(k+1)
Pp = A*P*A' + Q;    

K = Pp*H'*inv(H*Pp*H' + R); 

x = xp + K*(z - H*xp);
P = Pp - K*H*Pp;   

  
pos = x(1);
vel = x(2);
Px  = P ;