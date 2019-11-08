%% create a random signal(Test)
% time = (0:0.1:100)';
% iterations = 10;
% y1 = (zeros(length(time),1));
% 
% 
% for i =1:iterations
%     y1 = y1 + (sin(i*time));
% end
% 
% y2 = (8*sin(time));

%% real signal (Realtime)
%Step 1: drag and drop file on workspace 

%Step 2: Data preprocessing
y1 = pressure(:,1);
y2 = Pressure(:,1);


%Step 3:Import sampling rate (Change here)
Fs = 1;
Ts = 1/Fs;

%Step 3: Generate time domain
time1 = (0:Ts:Ts*(length(y1)-1))';
time2 = (0:Ts:Ts*(length(y2)-1))';

%Step 4: Convert to timeseries data
y1_timeseries = [time1 y1];
y2_timeseries = [time2 y2];




%% initialize a distance matrix
dist = zeros(length(y1), length(y2));
distance = zeros(length(y1), length(y2));

%find euclidean distance between y1 and y2 
for i = 1:length(y1)
    for j = 1:length(y2)
        dist(i,j) = sqrt((y2(j) - y1(i))^2);
        distance(i,j) = (y2(j) - y1(i))^2;
    end
end

%% visualise on heatmap
distances_map = heatmap(dist);

%% initialize accumulated_cost
accumulated_cost = zeros(length(y1), length(y2));


%% Forward Propagation - compute accumulated cost
%if the coord is (0,0), then the accumulated cost is the distance itself.
%if the coord has 0th column, then the accumulated cost would be the cost
%made from moving to that tile from one row before. 
%if the coord has 0th row, then the accumulated cost would be the cost made
%from moving to that tile from one column before.

accumulated_cost(1,1) = dist(1,1);

if (length(y1) > length(y2))
    maxlength = y1;
else 
    maxlength = y2;
end

for i = 2:length(y2)
    accumulated_cost(1,i) = accumulated_cost(1,i-1) + dist(1,i);
end

for i = 2:length(y1)
    accumulated_cost(i,1) = accumulated_cost(i-1,1)+dist(i,1);
end

for i = 2:length(y1)
    for j = 2:length(y2)
        accumulated_cost(i,j) = min(min(accumulated_cost(i-1,j-1),accumulated_cost(i,j-1)),accumulated_cost(i-1,j))+dist(i,j);
    end
end

%op1 = struct('show',false);
accumulated_cost_map = heatmap(accumulated_cost);

%% Backtracing
%We are currently going to backtrace the path from the destination to the
%start point. We are moving from coordinate (len (y1)-1, len(y2)-1) back to
%the start point (0,0).

allpath = [length(y1),length(y2)];

i = length(y1)-1;
j = length(y2)-1;

while((i>1) & (j>1))
    if(i==1)
        j = j - 1;
    
    elseif(j==1)
        i = i - 1;
    
    
    else
        if(accumulated_cost(i-1,j) == min(min(accumulated_cost(i-1,j), accumulated_cost(i,j-1)), accumulated_cost(i-1,j-1)))
            i = i - 1;
        
        elseif (accumulated_cost(i,j-1) == min(min(accumulated_cost(i-1,j), accumulated_cost(i,j-1)), accumulated_cost(i-1,j-1)))
            j = j - 1;
        
        else
            i = i - 1;
            j = j - 1;
        end
    allpath = vertcat(allpath, [i,j])
    end
end

allpath = vertcat(allpath,[0,0]);   

plot_dist = plot(allpath(:,1), allpath(:,2));

%overlay({accumulated_cost_map, plot_dist});

            


    


