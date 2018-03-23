% CONACYT
% Copyright (c) 2017, J. Irving Vasquez-Gomez,
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the <organization> nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL J. Irving Vasquez-Gomez BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% Program that test the method of optimal coverage path planning based on
% rotating calipers algorithm. 


% Configuration
n_vertices = 5;
polygon_radius = 100; %meters % experimentos en 100
rad_var = 1 ;%5
ang_var = 1 ;%1
dx = 20;
transl_spd = 10;
rot_spd = pi/4;

samplingx0 = -300;
samplingx1 = 300;
samplingy0 = -300;
samplingy1 = 300;

x_start = (samplingx1- samplingx0) * rand() + samplingx0;
y_start = (samplingy1- samplingy0) * rand() + samplingy0;
x_end = (samplingx1- samplingx0) * rand() + samplingx0;
y_end = (samplingy1- samplingy0) * rand() + samplingy0;

x_end = x_start
y_end = y_start

% get polygon
[M, Mshifted]= getConvexPolygon(n_vertices,polygon_radius,rad_var,ang_var);

%----- Optimal coverage path planning -----
tic;
% Compute Antipodal pairs
A = antipodalPoints(M)
[m, ~] = size(A);

% Graph polygon and antipodal points
%figure('Position',[10 100 500 500],'Renderer','zbuffer');
%axis equal; hold on;
%line([M(:,1)';Mshifted(:,1)'],[M(:,2)';Mshifted(:,2)'],'Color','k');
%title('Antipodal pairs');
%xlabel('East (x)'); ylabel('North (y)');
sz = 25; c = linspace(1,10,m);
%for i=1:m
%    scatter( M(A(i,1),1), M(A(i,1),2), sz, c(i), 'filled' );
%    scatter( M(A(i,2),1), M(A(i,2),2), sz, c(i), 'filled' );
%    line([M(A(i,1),1); M(A(i,2),1)],[M(A(i,1),2);M(A(i,2),2)],'Color',[rand() rand() rand()],'LineStyle','--');
%end
%hold off;
%%scatter(x_start, y_start, 25, 'filled');
%%scatter(x_end, y_end, 25, 'filled');

min_cost = Inf;
optimal_path = [];
best_antipodal_pair = 0;

for i=1:m 
    Path = bestPathForAntipodalPair(M, A(i,:), dx);
         
    %check if the path should be inverted
    FullPath1 = [x_start y_start; Path;  x_end y_end];
    FullPath2 = [x_start y_start; flipud(Path); x_end y_end];
    
    cost1 = timeCost2D(FullPath1, transl_spd, rot_spd, [x_start y_start 0]);
    
    cost2 =  timeCost2D(FullPath2, transl_spd, rot_spd, [x_start y_start 0]);
    
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
        
    %--------- Draw the best path for an antipodal pair
%     figure;
%     axis equal;
%     line([M(:,1)';Mshifted(:,1)'],[M(:,2)';Mshifted(:,2)'],'Color','k');
%     title('Best path for an antipodal pair');
%     ylabel('x(meters)');
%     xlabel('y(meters)');
%     hold on;
%     scatter( M(A(i,1),1), M(A(i,1),2), sz, c(i), 'filled' );
%     scatter( M(A(i,2),1), M(A(i,2),2), sz, c(i), 'filled' );
%     plot(FullPath(:,1), FullPath(:,2));
%     txt1 = ['cost = ', num2str(Cost(i))];
%     text(x_start,y_start,txt1);
%     hold off;
end

tiempo_vasq = toc


figure('Position',[10+500 100 500 500]);
axis equal;
line([M(:,1)';Mshifted(:,1)'],[M(:,2)';Mshifted(:,2)'],'Color','k');
title('Vasquez');
xlabel('East (x)');
ylabel('North (y)');
hold on;
i = best_antipodal_pair;
scatter( M(A(i,1),1), M(A(i,1),2), sz, c(i), 'filled' );
scatter( M(A(i,2),1), M(A(i,2),2), sz, c(i), 'filled' );
%line([M(A(i,1),1); M(A(i,2),1)],[M(A(i,1),2);M(A(i,2),2)],'Color',[rand() rand() rand()],'LineStyle','--');

%display('Vasquez cost')
%timeCost2D(optimal_path, transl_spd, rot_spd, [x_start y_start 0])
plot(optimal_path(:,1), optimal_path(:,2));
scatter(x_start, y_start, 25, 'filled');
scatter(x_end, y_end, 25, 'filled');
txt1 = ['cost = ', num2str(min_cost)];
text(x_start,y_start,txt1);
hold off;
    
    
% ------- Compare with Torres --------
tic
[PathTorres, PathTorres_counter, r_torres] = torres16(M, dx);
%check if the path should be inverted
FullPath1 = [x_start y_start; PathTorres; x_end y_end];
FullPath2 = [x_start y_start; flipud(PathTorres); x_end y_end];
FullPath3 = [x_start y_start; PathTorres_counter; x_end y_end];
FullPath4 = [x_start y_start; flipud(PathTorres_counter); x_end y_end];

cost1 = timeCost2D(FullPath1, transl_spd, rot_spd, [x_start y_start 0]);
cost2 = timeCost2D(FullPath2, transl_spd, rot_spd, [x_start y_start 0]);
cost3 = timeCost2D(FullPath3, transl_spd, rot_spd, [x_start y_start 0]);
cost4 = timeCost2D(FullPath4, transl_spd, rot_spd, [x_start y_start 0]);

torres_cost = cost1;
FullPathTorres = FullPath1;

if (cost2 < torres_cost)
    FullPathTorres = FullPath2;
    torres_cost = cost2;
end

if (cost3 < torres_cost)
    FullPathTorres = FullPath3;
    torres_cost = cost3;
end

if (cost4 < torres_cost)
    FullPathTorres = FullPath4;
    torres_cost = cost4;
end

time_torr = toc

%FullPathTorres = [x_start y_start; PathTorres; x_end y_end];
%display('Torres cost');
%timeCost2D(FullPathTorres, transl_spd, rot_spd, [x_start y_start 0])

figure('Position',[10+1000 100 500 500]);
axis equal;
line([M(:,1)';Mshifted(:,1)'],[M(:,2)';Mshifted(:,2)'],'Color','k');
title('Torres 16 Path');
xlabel('East (x)');
ylabel('North (y)');
hold on;
plot(FullPathTorres(:,1), FullPathTorres(:,2));
scatter(x_start, y_start, 25, 'filled');
scatter(x_end, y_end, 25, 'filled');
txt1 = ['cost = ', num2str(torres_cost)];
text(x_start,y_start,txt1);
hold off;