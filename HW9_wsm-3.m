%  William Murphy(SID#30640826),Matlab Script, april 2 2017, MIE124 Self Driving Car Simulation 
%% 
function [P_Crash_I,P_Crash_II,F_T_avg] = HW9_wsm

% This function  simulates cars driving through a four way intersection.For Part..
% one the cars have constant velocity and for part two, one direction of
% incoming cars accelerate. In both parts, crashes between cars and their positions are 
% are tracked.In the second part, the acceleration is controlled so as to
% minimize crashes and fuel. 
% 
%% Part I

%Initial Values 
t = linspace(0,1000,10000);
r_car = 1.5;                %meters 
m_car = 900;                %kg

% Initial Positions 
P_i_north = [2 -100]; 
P_i_south = [-2 100];
P_i_east = [-100 0];

% Calling Position subfunction
[P_N,N_N] = P_matrix(t,20,P_i_north(2),1);
[P_S,N_S] = P_matrix(t,-20,P_i_south(2),1);
[P_E,N_E] = P_matrix(t,15,P_i_east(1),0);


% Intitalizing Matrices for Crash Subfunction 
Crash_matrix = zeros(1,N_E-2);
D_min_matrix = zeros(1,N_E-2);

% Calling Crash Subfunction 
for l = 1:(N_E-2)          %Last two cars don't enter danger zone
   
    [Crash_matrix(l),D_min_matrix(l)] = crash_func(t,P_E(:,l),P_S,P_N,1);
end 

% Percentage of Crashed Cars
P_Crash_I = sum(Crash_matrix)/length(Crash_matrix)*100;

% Plotting 

% first graph 
hist(D_min_matrix)
title('Minimum Distance Matrix')
ylabel('Minimum Distance')
figure

%second graph 
crash_index = find(Crash_matrix == 1);
crash_vector = P_E(:,crash_index(1));
crash_pos_i = min(crash_vector(crash_vector >= -5));

crash_t = (crash_vector == crash_pos_i);
plot_south_cars = P_N(crash_t,:);
plot_north_cars = P_S(crash_t,:);

x1 = (-10:0.1:10);                                   %Vectors for plotting
y1 = zeros(1,length(x1));
y2 = (-200:0.1:200);
x2 = zeros(length(y2))+2;
y3 = (-200:0.1:200);
x3 = zeros(length(y3))-2;

plot(crash_pos_i,0,'ok',P_i_north(1),plot_north_cars,'ro',P_i_south(1),plot_south_cars,'go',x1,y1,'k-',x2,y2,'r-',x3,y3,'g-')
xlim([-10 10])
ylim([-200 200])
title('First Crash Intersection Simulation')
xlabel('meters')
ylabel('meters')

%% Part II

for i = 1:100
    
% Calling Position Subfunctions
[P_N_II,N_N_II] = P_matrix(t,20,P_i_north(2),1);
[P_S_II,N_S_II] = P_matrix(t,-20,P_i_south(2),1);

% Random Start Time
t_II = randi([250,750],1,1);

% Initial X_distance
x_dist = -100;

% Initial Velocity and Counting Variable
V = 15;
j = 1;
 
%While loop to simulate self driving car
while x_dist <= 5 

  % Calling Thruster Force Subfunction and Calc Drag Force
  Drag_Force(j) = (0.5*1.225)*(0.5*pi*(1.5)^2)*(V(j))^2;
  [Thruster_Force(j)] = Thrust_Optimization(V,P_N_II,P_S_II, x_dist(1), t_II,m_car); 

  % Calculating total force and acceleration 
  Total_Force(j) = Thruster_Force(j) - Drag_Force(j); 

  Acceleration(j) = Total_Force(j)/m_car;

  %Applying RK2
  h = 0.1;                                                    % time step(s)

  V_p = Acceleration(j)*h + V(j);                             % V prime for calulating next step drag 

  Drag_Force_p = (0.5*1.225)*(0.5*pi*(1.5)^2)*(V_p)^2;        % Drag Force Prime for next step accel

  Acceleration_p =  (Thruster_Force(j) - Drag_Force_p)/m_car; % Acceleration Prime for next step V

  V(j+1) = 0.5*(Acceleration(j)+ Acceleration_p)*h + V(j);    % Next Step Accel 

  x_dist(j+1,1) = 0.5*(V(j) + V(j+1))*h + x_dist(j,1);        % Next Step V


  % Update Counting and Time Variable 
  t_II(j+1,1) = t_II(j,1) + h;    
  j = j+1;
  
end 

% Calling Subfunction to Determine Crashes 
[Crash_matrix_II(i),D_min_matrix_II(i)] = crash_func(t_II,x_dist,P_S_II,P_N_II,2);

% Calculating Average Thrust 
Avg_Thruster_Force(i) = sum(Thruster_Force)/length(Thruster_Force);

end 

% Percentage of Crashed Cars
P_Crash_II = sum(Crash_matrix_II)/length(Crash_matrix_II)*100;

% Overall Average Thrust of 100 Trials

F_T_avg = sum(Avg_Thruster_Force)/length(Avg_Thruster_Force);

% Plotting 

figure 

%first graph 
hist(Avg_Thruster_Force)                 
title('Avg_Thruster_Force')
ylabel('Newtons')

figure

%second graph(first crash)
crash_index_II = find(Crash_matrix_II == 1);
crash_vector_II = P_E(:,crash_index_II(1));
crash_pos_i_II = min(crash_vector_II(crash_vector_II >= -5));

crash_t_II = (crash_vector_II == crash_pos_i_II);
plot_south_cars_II= P_N(crash_t_II,:);
plot_north_cars_II = P_S(crash_t_II,:);

x1 = (-10:0.1:10);                                   %Vectors for plotting
y1 = zeros(1,length(x1));
y2 = (-200:0.1:200);
x2 = zeros(length(y2))+2;
y3 = (-200:0.1:200);
x3 = zeros(length(y3))-2;

plot(crash_pos_i_II,0,'ok',P_i_north(1),plot_north_cars_II,'ro',P_i_south(1),plot_south_cars_II,'go',x1,y1,'k-',x2,y2,'r-',x3,y3,'g-')
xlim([-10 10])
ylim([-200 200])
title('First Crash Intersection Simulation 2')
xlabel('meters')
ylabel('meters')

% Third Graph (first success)
figure
crash_index_II = find(Crash_matrix_II == 0);
crash_vector_II = P_E(:,crash_index_II(1));
crash_pos_i_II = min(crash_vector_II(crash_vector_II >= -5));

crash_t_II = (crash_vector_II == crash_pos_i_II);
plot_south_cars_II= P_N(crash_t_II,:);
plot_north_cars_II = P_S(crash_t_II,:);

plot(crash_pos_i_II,0,'ok',P_i_north(1),plot_north_cars_II,'ro',P_i_south(1),plot_south_cars_II,'go',x1,y1,'k-',x2,y2,'r-',x3,y3,'g-')
xlim([-10 10])
ylim([-200 200])
title('First Success Intersection Simulation 2')
xlabel('meters')
ylabel('meters')



end 

%% Position Matrix Subfunction 
function [P,N] = P_matrix(t,V,P_i,ns)
 
t_increment = 0.1;                              %change in t vector

if ns == 0
 %% East-West Position      
 
 t_car_increment = 5;                           %time between cars 
 
 far_car_pos = -(t(end)*V);                     %farthest car position

 N = t(end)/t_car_increment+1;                  %Number of Cars 

 dist_from_P_i = linspace(0,far_car_pos,(N));   %Creating All other cars

 ew_initial_car_position = dist_from_P_i + P_i; %Adding first car to other cars
 
 % Initialzing Position Matrix and Counting Variable 
 P = zeros(10001,201);
 P(1,:) = ew_initial_car_position;
 j = 1;

 % While Loop to Update Cars Position in Time/Create P_matrix
 while t(j) <= 1000
   P(j+1,:) = P(j,:) + V*t_increment;
     
   j = j+1;       
   if j > 10000    
      break
   else 
   end 
    
 end 


else

 %% North-South Position 

 %initilzing counting and position variables
 P(1,1) = P_i;
 t_car_increment = 0;
 j = 1;

% While Loop to Determine Time and Distance Increment Between Cars
 while sum(t_car_increment) <= 1000
   
   t_car_increment(j) = randi([3,5],1);
   Distance_from_P_i(j) = -V*t_car_increment(j);
   P(1,j+1) = P(1,j)+ Distance_from_P_i(j);
  
   j = j+1;
 end 
 
 %Initializing Counting Variables and Position Matrix 
 k = 1;
 P_initialize = zeros(10000,length(P));
 P = [P ; P_initialize];

 % While Loop to Update Cars Position in Time/Create P_matrix
 while t(k) <= 1000
  P(k+1,:) = P(k,:) + V*t_increment;
     
  k = k+1;       
  if k > 10000    
     break
  else 
  end   
    
 end 

 N = length(Distance_from_P_i)+1;

end
end 


    
%% Crash Subfunction

 function [Crash,D_min] = crash_func(t,x,P_S,P_N,part)
  
  %finding when car is in danger zone 
  if part == 1
   danger_car_position  = x(x <= 5 & x >= -5);
  
   danger_P_N = P_N((x <= 5 & x >= -5),:);
   danger_P_S = P_S((x <= 5 & x >= -5),:);
  else 
   danger_index_x = find(x <= 5 & x >= -5);
   danger_index_P_mat = (t(1)*10-1) + find(x <= 5 & x >= -5);
  
   danger_car_position = x(danger_index_x);
   danger_P_N = P_N(danger_index_P_mat,:);
   danger_P_S = P_S(danger_index_P_mat,:);
  end 
  
  N_distance = sqrt((2-danger_car_position).^2 + (danger_P_N).^2);
  S_distance = sqrt((-2-danger_car_position).^2 + (danger_P_S).^2);
  
  D_matrix = [N_distance S_distance];
  min_row = min(D_matrix);
  D_min = min(min_row);
  
  if D_min <= 3 
      Crash = 1;
  else 
      Crash = 0;
  end
  
 end


%% Thrust Optimization Subfunction 

function [Force] = Thrust_Optimization(V,P_N_II,P_S_II, x_dist, t_II,m_car)  

j = 1;

while x_dist <= 5 

  % Calling Thruster Force Subfunction and Calc Drag Force
  Drag_Force(j) = (0.5*1.225)*(0.5*pi*(1.5)^2)*(V(j))^2;
  Acceleration(j) = -Drag_Force(j)/m_car;
  %Applying RK2
  h = 0.1;                                                    % time step(s)
  V_p = Acceleration(j)*h + V(j);                             % V prime for calulating next step drag 
  Drag_Force_p = (0.5*1.225)*(0.5*pi*(1.5)^2)*(V_p)^2;        % Drag Force Prime for next step accel
  Acceleration_p =  - (Drag_Force_p)/m_car;                    % Acceleration Prime for next step V
  V(j+1) = 0.5*(Acceleration(j)+ Acceleration_p)*h + V(j);    % Next Step Accel 
  x_dist(j+1,1) = 0.5*(V(j) + V(j+1))*h + x_dist(j,1);        % Next Step V
  % Update Counting and Time Variable 
  t_II(j+1,1) = t_II(j,1) + h;    
  j = j+1;
  
end 

%Callign crash function to determine crash 
[Crash,D_min] = crash_func(t_II,x_dist,P_S_II,P_N_II,2);

if Crash == 0 
    Force = 0;
else 
    Force = 1000;  
end 

end 
 
 % The thinking behind this thrust minimizing subfunction is to use
 % somewhat of a brute force method in order to keep it simple but still
 % effective. In this function, the positions of the north and southbound
 % cars are used to see if the car will crash or not without the influence
 % of thrusters. If it will not then no thrusters are used and fuel is
 % conserved at a moderate rate considering the normal crash percentage is
 % around 20. However, if it is predicted that the car will crash, then a
 % force of 1000N is applied throughout the entire time in order to get it
 % through the intersection fast enough to avoid most crashes. Also this
 % force minimizes fuel to a certain extent. Overall the results of this
 % function are relatively good, as the the avgerage fuel use is around 200
 % and the crash rate is around 5%, which is about one fourth of the normal
 % of the part I rate of around 20%. 
 %   
