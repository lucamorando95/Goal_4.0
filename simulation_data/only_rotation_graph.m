%% Only Rotation Graphs 
%% Graficate on Rototraslation  experiment
close all
load_multiple_photo = 0;
title_FontSize = 38;
label_FontSize = 34;
legend_FontSize = 28;


folder = '/home/lucamora/PHD/paper_for_journal_publication/Test_data/test_GPS_coo_rototraslated/only_rotation/Exp_3';




python = 0;
% State Estimation in GF
fileID = fopen(strcat(folder,'/a_est_GF.txt'),'r');
formatSpec = '%f';
a_est_GF = fscanf(fileID,formatSpec);
fclose(fileID);

fileID = fopen(strcat(folder ,'/c_est_GF.txt'),'r');
formatSpec = '%f';
c_est_GF = fscanf(fileID,formatSpec);
fclose(fileID);




% Drone Position in frame HOME
fileID = fopen(strcat(folder ,'/x_pos.txt'),'r');
formatSpec = '%f';
x_pos = fscanf(fileID,formatSpec);
fclose(fileID);

fileID = fopen(strcat(folder ,'/y_pos.txt'),'r');
formatSpec = '%f';
y_pos = fscanf(fileID,formatSpec);
fclose(fileID);




%Offset From GPS LINE IN DJI
fileID = fopen(strcat(folder ,'/error_from_GPS_line.txt'),'r');
formatSpec = '%f';
error_from_GPS_line = fscanf(fileID,formatSpec);
fclose(fileID);

%Offset From Vision LINE IN DJI
fileID = fopen(strcat(folder ,'/error_from_vision_line.txt'),'r');
formatSpec = '%f';
error_from_vision_line = fscanf(fileID,formatSpec);
fclose(fileID);


a_line_GF = zeros(length(a_est_GF),1);
c_line_GF = zeros(length(c_est_GF),1);

%a_line_GF = -0.36+a_line_GF;


%Line to define the separation between the two arrays
line1_s_y = [-2.0; 2];
line1_s_x = [1154; 1154];
line2_s_y = [-2.0; 2];
line2_s_x = [1469; 1469];


a_est_GF = [a_est_GF(1:1164); a_est_GF(1164)*ones(305,1);a_est_GF(1460:end)];
c_est_GF = [c_est_GF(1:1164); c_est_GF(1164)*ones(305,1);c_est_GF(1460:end)];

% a_est_GF = [a_est_GF(1: 690); a_est_GF(690 + 165: end)];
% c_est_GF = [c_est_GF(1: 690); c_est_GF(690 + 165: end)];


figure
%Decommentare solo con RGB
% a_est_GF = a_est_GF(1:6200);
% a_est_GF(2500) = 0;
% c_est_GF = c_est_GF(1:6200);
% a_est_GF(1885) = 0;


subplot(2, 1, 1)
%a_est_GF(1885) = 0;
%a_est_GF(1084) = 0;
p = plot(a_est_GF);
p.LineWidth = 1.5;
% hold on
% p = plot(a_line_GF)
% p.LineWidth = 1.5;
hold on

p = plot(line1_s_x, [0.4, -0.2]);
p.LineWidth = 1.5;
p.LineStyle = '-.';
p.Color = [1 0 0];
hold on

p = plot(line2_s_x, [0.4, -0.2]);
p.LineWidth = 1.5;
p.LineStyle = '-.';
p.Color = [1 0 0];


naam = ['PV Array Midline Parameter Estimation'];
t = title(naam,'interpreter', 'latex');
t.FontSize = title_FontSize;
l = xlabel('control steps', 'interpreter', 'latex');
l.FontSize = label_FontSize;
l = ylabel('value','interpreter', 'latex');
l.FontSize = label_FontSize;
l = legend('$a^W$', 'PV array midline', '','Waypoints ', 'interpreter','latex');
l.FontSize = legend_FontSize;
ax = gca;
ax.FontSize =25;


subplot(2, 1, 2)
%Decommentare solo con RGB

% c_est_GF = c_est_GF(1: 5140);
% a = ones(1035,1) * c_est_GF(5140);
% c_est_GF = [c_est_GF;a];
p = plot(c_est_GF);
p.LineWidth = 1.5;
hold on

p = plot(line1_s_x, line1_s_y);
p.LineWidth = 1.5;
p.LineStyle = '-.';
p.Color = [1 0 0];

hold on

p = plot(line2_s_x, line2_s_y);
p.LineWidth = 1.5;
p.LineStyle = '-.';
% hold on
% p = plot(c_line_GF)
% p.LineWidth = 1.5

naam = ['PV Array Midline Parameter Estimation'];
t = title(naam,'interpreter', 'latex');
t.FontSize = title_FontSize;
l = xlabel('control steps', 'interpreter', 'latex');

l.FontSize = label_FontSize;
l = ylabel('meters','interpreter', 'latex');
l.FontSize = label_FontSize;
l = legend('$b^W$', 'PV array midline', '','Waypoints ', 'interpreter','latex');
l.FontSize = legend_FontSize;
ax = gca;
ax.FontSize =25;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('path.mat')
        
        a = find(x_pos == -27.3804);
        b = y_pos(a);
        
        x_pos_to_save = flip(x_pos_to_save);
        % x_pos_to_save = x_pos_to_save(find(x_pos_to_save == -27.5249) : end);
        % y_pos_to_save = y_pos_to_save(find(x_pos_to_save == -27.5249) : end);
        
        x_pos_to_save = x_pos_to_save(1 : find(x_pos_to_save == 55.9631));
        y_pos_to_save = y_pos_to_save(1 : find(x_pos_to_save == 55.9631));
        
        y_pos_to_save = y_pos_to_save - y_pos_to_save(1);
        x_pos = [x_pos(1:a); x_pos_to_save;  x_pos(find(x_pos == 56.0971):end)];
        y_pos = [y_pos(1:a);  y_pos(a)+ y_pos_to_save;  y_pos(find(y_pos == 10.7879):end)];
        
        %True Waypoints Position
        P1_start = [vpa(-26.5367453628); vpa(10.935672313)];
        P1_end = [vpa(56.7886758395902); vpa(10.895637241)];
        %P1_end = [vpa(-16.7886758395902); vpa(10.056757384)];
        
        P2_start = [vpa(56.8785473703139); vpa(15.994572124)];
        %P2_start = [vpa(-16.8785473703139); vpa(15.9328472193)];
        P2_end = [vpa(-28.48304882148); vpa(15.9237711834)];
        
        GPS_true_path_x = [P1_start(1), P1_end(1), P2_start(1), P2_end(1)];
        GPS_true_path_y = [P1_start(2), P1_end(2), P2_start(2), P2_end(2)];
        
        %ROTOTRASLATED GPS ---> ONLY ROTATION EXP3
        
        P = [P1_start, P1_end, P2_start, P2_end];
        % rotate local coordinates


       % Place rotational frame in P1 start
        P = P - P1_start;
        tx = 0.0; %1.0;
        ty = 0.0; %0.8
        gamma = 0.1; % rad
        
        R_z = [cos(gamma) -sin(gamma) tx ;...
            sin(gamma) cos(gamma) ty];
        
        
        
        
        for ii = 1 : 4
            
            
            v1 = cos(gamma)*P(1,ii) - sin(gamma)*P(2, ii)
            v2 = sin(gamma) * P(1,ii) + cos(gamma)*P(2, ii)
            P_new(:,ii) = [v1;v2]
            P_new(1, ii) = P_new(1, ii) + P1_start(1)
            P_new(2, ii) = P_new(2, ii) + P1_start(2)
            
        end
        %
        % %
        % P_new(1,3)  = P1_start(1) + P_new(1,3);
        % P_new(1,4) = P1_start(1) + P_new(1,4);
        % P_new(2,3) = P1_start(1) *
%         GPS_rotated_path_x = [P_new(1,1), P_new(1,2), P_new(1,3), P_new(1,4)];
%         GPS_rotated_path_y = [P_new(2,1), P_new(2,2), P_new(2,3), P_new(2,4)];
        
         GPS_rotated_path_x = [-26.5367,56.3724, 55.8753, -28.9711];
         GPS_rotated_path_y = [10.953,  19.2301, 24.2241, 15.713];
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
figure
p = plot(x_pos, y_pos);
p.LineWidth = 1.5;
hold on
p1 = plot(GPS_true_path_x, GPS_true_path_y );
% p1.Marker = 'square';
p1.LineStyle = '-.';
p1.LineWidth = 2;
p1.Color = 	[1 0 0];

% hold on 
% p1 = plot(GPS_true_path_x_PV2, GPS_true_path_y_PV2 );
% % p1.Marker = 'square';
% p1.LineStyle = '-.';
% p1.LineWidth = 2;
% p1.Color = 	[1 0 0];
% % p1.MarkerSize = 20;
% % p1.MarkerEdgeColor = 'g';
% p1.MarkerFaceColor = [0.5,0.5,0.5];

%hold on
% p1 = plot(Gps_path_P2_start_end_X, Gps_path_P2_start_end_Y);
% p1.Marker = 'square';
% p1.LineStyle = '-.';
% p1.MarkerSize = 20;
% p1.MarkerEdgeColor = 'r';
% p1.MarkerFaceColor = [0.5,0.5,0.5];

hold on
p1 = plot(GPS_rotated_path_x, GPS_rotated_path_y );
p1.Marker = 'square';
p1.LineStyle = '-.';
p1.LineWidth = 0.01;
p1.Color = [1 1 1];
p1.MarkerSize = 20;
p1.MarkerEdgeColor = 'g';
p1.MarkerFaceColor = [0.5,0.5,0.5];
%
% hold on
% p1 = plot(Gps_path_P2_start_end_X, Gps_path_P2_start_end_Y);
% p1.Marker = 'square';
% p1.LineStyle = '-.';
% p1.MarkerSize = 20;
% p1.MarkerEdgeColor = 'r';
% p1.MarkerFaceColor = [0.5,0.5,0.5];

% hold on
% plot(P2_start_X, P2_start_Y-9.5, 'sq', 'MarkerSize', 30);
% hold on
% plot(P2_end_X, P2_end_Y -9.5, 'sq', 'MarkerSize', 30);
%
% Hold on circle near waypoints
naam = ['UAV 2D Position in World Frame'];
t= title(naam,'interpreter','latex');
title_FontSize = 36;
t.FontSize = title_FontSize;
l = xlabel('meters','interpreter','latex');
l.FontSize = label_FontSize;
l= ylabel('meters','interpreter','latex');
%set(gca,'FontSize',20)
l.FontSize = label_FontSize;
l = legend('UAV Path','PV array midline', '','Waypoints ', 'interpreter','latex');
l.FontSize = legend_FontSize;
ax = gca;
ax.FontSize =25;
axis equal




%Graficate the navigation error
% Error distance from true line


err_y_P1_start_P1_end = P1_start(2) - y_pos(1:length(y_pos)/2);

%y_pos_err_2 = [y_pos(1344:end)];
err_y_P2_start_P2_end = P2_start(2) - y_pos(length(y_pos)/2 + 1: end);

err_y = [err_y_P1_start_P1_end; err_y_P2_start_P2_end];


mean_err = mean(abs([err_y(442:line1_s_x(1)); err_y(line2_s_x(1):end)]));
std_err_y = std(abs([err_y(442:line1_s_x(1)); err_y(line2_s_x(1):end)]));



err_y = [err_y(1: line1_s_x(1)); err_y(1151) * ones(line2_s_x(1) - line1_s_x(1),1); err_y(line2_s_x(1): end)];
err_y(1154) = err_y(1151);
err_y(1153) = err_y(1151);

figure
p = plot(err_y);
p.LineWidth = 1.5;
hold on
p = plot(line1_s_x, [-2.0 12]);
p.LineWidth = 1.5;
p.LineStyle = '-.';
p.Color = [1 0 0];

hold on
p = plot(line2_s_x,  [-2.0 12]);
p.LineWidth = 1.5;
p.LineStyle = '-.';
p.Color = [1 0 0];




naam = ['Navigation Error $\xi$'];
t = title(naam,'interpreter', 'latex');
t.FontSize = title_FontSize;
l = xlabel('control steps', 'interpreter', 'latex');
l.FontSize = label_FontSize;
l = ylabel('meters','interpreter', 'latex');
l.FontSize = label_FontSize;
l = legend('Error $\xi$', 'End PV array line', 'Start PV array line', 'interpreter', 'latex');
l.FontSize = legend_FontSize;
ax = gca;
ax.FontSize =25;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control error e
error_from_vision_line = [0*error_from_vision_line(1:494);  error_from_vision_line(495:end)];
 mean_error = error_from_vision_line;
    % mean_error = [error_from_vision_line_1; error_from_vision_line_2];
    mean_e = mean(abs(mean_error))
    std_e = std(abs(mean_error))
    figure
    p = plot(error_from_vision_line);
    p.LineWidth = 1.5;
    naam = ['Control Error $e$'];
    t = title(naam,'interpreter', 'latex');
    t.FontSize = title_FontSize;
    l = xlabel('control steps', 'interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = ylabel('meters','interpreter', 'latex');
    l.FontSize = label_FontSize;
    ax = gca;
    ax.FontSize =25
    
    
    
    % Navigation Error = 0.32161+-0.3457
    % Control Error = 0.048 +-  0.0552








