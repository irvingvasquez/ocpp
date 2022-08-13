%% Example Title
% Summary of example objective

%% Section 1 Title
% Description of first code block
polygon_file = "C:\Users\vagop\projects\sweep_cpp\out\target_rcpp_room.txt";
home_point_file = "C:\Users\vagop\projects\sweep_cpp\out\p_i_a_rcpp.txt";
dx = 50;
[poly, poly_s] = loadPolygonCSV(polygon_file);
home_point = readmatrix(home_point_file);
home_point

[optimal_path] = RCPP(poly, dx, home_point, home_point, 1.0, 0.1)

%% Section 2 Title
% Description of second code block
%line([poly(:,1)';poly_s(:,1)'],[poly(:,2)';poly_s(:,2)'],'Color','k');

figure('Position',[10+500 100 500 500]);
axis equal;
line([poly(:,1)';poly_s(:,1)'],[poly(:,2)';poly_s(:,2)'],'Color','k');
title('Vasquez');
xlabel('East (x)');
ylabel('North (y)');
hold on;
i = best_antipodal_pair;
%scatter( poly(A(i,1),1), poly(A(i,1),2), sz, c(i), 'filled' );
%scatter( poly(A(i,2),1), poly(A(i,2),2), sz, c(i), 'filled' );
%line([poly(A(i,1),1); poly(A(i,2),1)],[poly(A(i,1),2);poly(A(i,2),2)],'Color',[rand() rand() rand()],'LineStyle','--');

%display('Vasquez cost')
%timeCost2D(optimal_path, transl_spd, rot_spd, [x_start y_start 0])
plot(optimal_path(:,1), optimal_path(:,2), '-o');
scatter(x_start, y_start, 25, 'filled');
scatter(x_end, y_end, 25, 'filled');
txt1 = ['cost = ', num2str(min_cost)];
text(x_start,y_start,txt1);
hold off;

path_file = "C:\Users\vagop\projects\sweep_cpp\out\path.txt";
writematrix(optimal_path, path_file);