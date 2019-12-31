% This program demonstrates Q Learning and how a sensor, radar and GPS, based approach can
% be more beneficial than using absolute position in maze. The robot moves
% from the top left corner to the bottom right hand corner. The dynamic
% obstacle, human,  starts in the middle of the maze and moves randomly
% every time step. A sparse data structure is used to save space. 
function Sparse_Qlearning
clear all;
close all;

% Define the maze
maze = [0,0,0,0,0,0,0,0,0,0,0,0;
        0,1,1,1,0,1,1,1,1,1,1,0;
        0,1,1,1,1,1,1,1,1,1,1,0;
        0,1,1,1,1,1,1,0,0,0,1,0;
        0,1,1,0,1,1,1,0,1,1,1,0;
        0,1,1,0,1,0,1,1,0,1,1,0;
        0,1,1,0,0,0,1,1,1,1,1,0;
        0,1,1,1,1,1,1,0,0,1,1,0;
        0,1,1,1,0,1,1,1,1,1,1,0;
        0,1,1,1,0,1,1,1,1,1,1,0;
        0,1,1,1,0,1,1,0,1,1,2,0;
        0,0,0,0,0,0,0,0,0,0,0,0];

% Variable definition
Q = [0];
all_steps = [];
all_Q_size = [];
no_dp = [];
dynamic_ob_x = 6;
dynamic_ob_y = 7;
final_x = 11;
final_y = 11;
% Learning rate settings
alpha = 0.1; 
gamma = 0.9;

% Max number of times Q Learning will re-iterate
max_iter = 200;

L = 10; % distance
n=5;
% Determine distance from goal
dist = sqrt((1 - 10).^2 + (1 - 10).^2);

% Discretize the distance to determine state d
distMax = sqrt(2)*(L-1); % The largest distance from goal possible for this grid
edges = 0:distMax/n:distMax; % Edges for discretizing dist
d = discretize(dist, edges); % Determine between which edges dist lies
d = d-1; % To have range of d be from 0 to n-1

% Q Learning
for i=1:max_iter   
    % Initial sensor readings for robot. NSEW
    row_encoding = strcat('100','011','001','001');
    state = bin2dec(row_encoding);
    
    curr_x = 2;
    curr_y = 2;
    goal_dist = d;          
    curr_Q_index = -1;
    prev_Q_index = 1;
    steps = 0;
    done = 0;
    
    while done ~= 1
        % Choose max value in sparse matrix
        if Q(1) == 0
            Q = [state,goal_dist];
            action = [0,0,0,0];
        end
        % Check for state and select an action
        next_action = -1;
        for i=1:size(Q,1)
            % Check to see if state already exists
            if Q(i,1) == state & Q(i,2) == goal_dist
                [val,next_action] = max(action(i,:));
                [index,next_action] = find(action(i,:) == val);
                
                if size(next_action,2) ~=1
                    rng_index = randi([1,size(next_action,2)],1);
                    next_action = next_action(rng_index);
                end
                curr_Q_index = i;
                break;
            end
        end
        % Next_action was not found so add another row/col
        if next_action == -1
            Q = [Q;state,goal_dist];
            action = [action;0,0,0,0];
            curr_Q_index = size(action,1);
            next_action = randi([1,4],1);
        end
         
        % Set the epsilon value where a percentage of the time a random
        % action is taken instead
        random_num = rand;
            epsilon = 0.001;
        if random_num <= epsilon
            next_action = randi(4);
        end
      
        % Move the robot.
        next_x = -1;
        next_y = -1;
        % Top = 1
        if (next_action == 1)
            next_x = curr_x-1;
            next_y = curr_y;
        % Left = 2
        elseif (next_action == 2)
            next_x = curr_x;
            next_y = curr_y-1;
        % Right = 3
        elseif (next_action == 3)
            next_x = curr_x;
            next_y = curr_y+1;
        % Down = 4
        elseif (next_action == 4)
            next_x = curr_x+1;
            next_y = curr_y;
        end
        
       % Check if next_action will hit a wall, goal or valid spot
       % Wall
       if maze(next_x,next_y) == 0
           rewardVal = -10;
       % Valid spot
       elseif maze(next_x,next_y) == 1
           rewardVal = -1;
       % Goal
       else
           rewardVal = 100;
           done = 1;
       end
        
       % Robot hit wall move it back
        if (rewardVal ~= -10)
            curr_x = next_x;
            curr_y = next_y;
        end
        
        % Sensor will generate next state
        if (maze(curr_x-1,curr_y) == 0)
            north_state = '000';
        elseif(maze(curr_x-2,curr_y) == 0)
            north_state = '010';
        else
            north_state = '001';
        end
        if (maze(curr_x+1,curr_y) == 0)
            south_state = '000';
        elseif(maze(curr_x+2,curr_y) == 0)
            south_state = '010';
        else
            south_state = '001';
        end
        if (maze(curr_x,curr_y-1) == 0)
            west_state = '000';
        elseif(maze(curr_x,curr_y-2) == 0)
            west_state = '010';
        else
            west_state = '001';
        end
        if (maze(curr_x,curr_y+1) == 0)
            east_state = '000';
        elseif(maze(curr_x,curr_y+2) == 0)
            east_state = '010';
        else
            east_state = '001';
        end
        
        row_encoding = strcat(north_state,south_state,east_state,west_state);
        state = bin2dec(row_encoding);
        
        % Goal distance state (GPS)
        dist = sqrt((curr_x-1 - 10).^2 + (curr_y-1 - 10).^2);
        d = discretize(dist, edges); 
        goal_dist = d-1; 
        
          if(rewardVal == -1)
              % Find curr_Q_index if it went to a valid spot  
              curr_Q_index = -1;
              for i=1:size(Q,1)
                % Check to see if row and 
                if Q(i,1) == state & Q(i,2) == goal_dist
                    curr_Q_index = i;
                end
              end
              % Couldn't find it in table
              if curr_Q_index == -1
                  Q = [Q;state,goal_dist];
                  action = [action;0,0,0,0];
                  curr_Q_index = size(action,1);
              end
          end

          % Bellman Equation for Q Learning
          action(prev_Q_index,next_action) = action(prev_Q_index,next_action) + alpha*(rewardVal+gamma*max(action(curr_Q_index)) - action(prev_Q_index,next_action));

          prev_Q_index = curr_Q_index;
          steps = steps + 1;

          %update dynamic obstacle
          next_dynamic_ob_x = -1;
          next_dynamic_ob_y = -1;
          ob_action = randi(4);
          if (ob_action == 1)
            next_dynamic_ob_x = dynamic_ob_x-1;
            next_dynamic_ob_y = dynamic_ob_y;
          % Left = 2
          elseif (ob_action == 2)
            next_dynamic_ob_x = dynamic_ob_x;
            next_dynamic_ob_y = dynamic_ob_y-1;
          % Right = 3
          elseif (ob_action == 3)
            next_dynamic_ob_x = dynamic_ob_x;
            next_dynamic_ob_y = dynamic_ob_y+1;
          % Down = 4
          elseif (ob_action == 4)
            next_dynamic_ob_x = dynamic_ob_x+1;
            next_dynamic_ob_y = dynamic_ob_y;
          end

        % Robot hit wall or robot or goal
        if~((maze(next_dynamic_ob_x,next_dynamic_ob_y) == 0) || (next_dynamic_ob_x == curr_x && next_dynamic_ob_y == curr_y))
            maze(dynamic_ob_x,dynamic_ob_y) = 1;
            dynamic_ob_x = next_dynamic_ob_x;
            dynamic_ob_y = next_dynamic_ob_y;
            maze(dynamic_ob_x,dynamic_ob_y) = 0;
            maze(final_x,final_y) = 2;
        end

        % Display maze
        graph_maze = maze;
        graph_maze(curr_x,curr_y)=3;
        graph_maze(dynamic_ob_x,dynamic_ob_y) = 4;
        % Comment out to generate graph
        %imagesc(graph_maze);
        %drawnow;
    end
    all_steps = [all_steps,steps];
    Q_size = size(Q,1)*size(Q,2)*4;
    all_Q_size = [all_Q_size,Q_size];
    no_dp = [no_dp,2025];
end

% Generate figures
figure
q = plot(all_steps);
q.LineWidth = 2;

figure
q = plot(all_Q_size);
q.LineWidth = 2;
hold on
q = plot(no_dp,'Color','Red');
q.LineWidth = 2;

xlabel('Episode');
ylabel('Size of Data Structure');
title('Average Size of Data Structure');
end
