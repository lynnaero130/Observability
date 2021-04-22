function x = Observer(imu,uwb,x0,dt)
%This function is observer.
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt,;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
  
  B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];

 
 x(:,1) = x0;
 
 for i = 1:size(imu,2)

%   x(:,i+1) = A*x(:,i) + B*imu(:,i) + (uwb(i) - norm(x(1:3,i)))*[x(1:3,i)'/uwb(i) 0 0 0]';
    x(:,i+1) = A*x(:,i) + B*imu(:,i) + (uwb(i) - norm(x(1:3,i)))*[x(1:3,i)'/uwb(i) 0 0 0]';
    
end

