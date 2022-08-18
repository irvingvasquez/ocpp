

function [optimal_path, min_cost] = RCPP(M, dx, starting_point, ending_point, translation_speed, rot_spd)
% Rotating calipers path planner 
% 
% M : Polygon
% dx distance between flight lines
% translation_speed and rot_spd are used to estimate the cost of a path

A = antipodalPoints(M);
[m, ~] = size(A);

x_start = starting_point(1);
y_start = starting_point(2);
x_end = ending_point(1);
y_end = ending_point(2);

min_cost = Inf;
optimal_path = [];
best_antipodal_pair = 0;

for i=1:m 
    Path = bestPathForAntipodalPair(M, A(i,:), dx);
         
    %check if the path should be inverted
    FullPath1 = [x_start y_start; Path;  x_end y_end];
    FullPath2 = [x_start y_start; flipud(Path); x_end y_end];
    
    cost1 = timeCost2D(FullPath1, translation_speed, rot_spd, [x_start y_start 0]);
    
    cost2 =  timeCost2D(FullPath2, translation_speed, rot_spd, [x_start y_start 0]);
    
    if (cost1 < cost2)
       FullPath = FullPath1;
       Cost(i) = cost1;
    else
       FullPath = FullPath2;
       Cost(i) = cost2;
    end
    
    if Cost(i)<min_cost
        min_cost = Cost(i);
        optimal_path = FullPath;
        best_antipodal_pair = i;
    end
end

end

