%Real Experiment: Only RGB 

close all
load_multiple_photo = 0;
title_FontSize = 38;
label_FontSize = 34;
legend_FontSize = 28;

%Giiusto Pero va tagliato per fare solo singolo array
folder = '/home/lucamora/PHD/paper_for_journal_publication/Test_data/Test_Predosa/Exp_2';




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







%Line to define the separation between the two arrays
line1_s_y = [-2.0; 2];
line1_s_x = [1324; 1324];
line2_s_y = [-2.0; 2];
line2_s_x = [1554; 1554];




figure
%Decommentare solo con RGB
a_est_GF = a_est_GF(1:6200);
a_est_GF(2500) = 0;
c_est_GF = c_est_GF(1:6200);
a_est_GF(1885) = 0;
c_est_GF(5156:end) = c_est_GF(5155)



subplot(2, 1, 1)
%a_est_GF(1885) = 0;
%a_est_GF(1084) = 0;
p = plot(a_est_GF);
p.LineWidth = 1.5;
% hold on
% p = plot(a_line_GF)
% p.LineWidth = 1.5;


% % % hold on
% % % 
% % % p = plot(line1_s_x, [-0.1, 0.06]);
% % % p.LineWidth = 1.5;
% % % p.LineStyle = '-.';
% % % p.Color = [1 0 0];
% % % 
% % % hold on
% % % 
% % % p = plot(line2_s_x, [-0.1, 0.06]);
% % % p.LineWidth = 1.5;
% % % p.LineStyle = '-.';
% % % p.Color = [1 0 0];



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



% hold on
% 
% p = plot(line1_s_x, [-0.6, 0.2]);
% p.LineWidth = 1.5;
% p.LineStyle = '-.';
% p.Color = [1 0 0];
% 
% hold on
% 
% p = plot(line2_s_x, [-0.6, 0.2]);
% p.LineWidth = 1.5;
% p.LineStyle = '-.';
% p.Color = [1 0 0];



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





GPS_true_path_x = [-33.0779, 40.4964];
GPS_true_path_y = [10.8333, 10.953];

x_pos = x_pos(1:6183);
y_pos = y_pos(1:6183);

figure
p = plot(x_pos, y_pos);
p.LineWidth = 1.5;
hold on
p1 = plot(GPS_true_path_x, GPS_true_path_y );
p1.Marker = 'square';
p1.LineStyle = '-.';
p1.LineWidth = 2;
p1.MarkerSize = 20;
p1.MarkerEdgeColor = 'g';
p1.MarkerFaceColor = [0.5,0.5,0.5];

%hold on
% p1 = plot(Gps_path_P2_start_end_X, Gps_path_P2_start_end_Y);
% p1.Marker = 'square';
% p1.LineStyle = '-.';
% p1.MarkerSize = 20;
% p1.MarkerEdgeColor = 'r';
% p1.MarkerFaceColor = [0.5,0.5,0.5];

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







% Misalignment Error 
% Error distance from true line
err_y_P1_start_P1_end = GPS_true_path_y(2) - y_pos;
err_y = err_y_P1_start_P1_end(1:6183);
%y_pos_err_2 = [y_pos(1344:end)];
% err_y_P2_start_P2_end = P2_start(2) - y_pos(length(y_pos)/2 + 1: end);
% 
% err_y = [err_y_P1_start_P1_end; err_y_P2_start_P2_end];
% 
% err_y = [err_y(1:1189); err_y(1189) * ones(238,1); err_y(1427:end)];

%             y_pos_err_1 = [y_pos(354:1343)];
%             err_y_P1_start_P1_end =P1_start(2) - y_pos_err_1;
%
%             y_pos_err_2 = [y_pos(1344:end)];
%             err_y_P2_start_P2_end = P2_start(2) - y_pos_err_2;
%
%             err_y = [err_y_P1_start_P1_end; err_y_P2_start_P2_end];
%
%


figure
p = plot(err_y);
p.LineWidth = 1.5;


% Hold on circle near waypoints
naam = ['Navigation Error $\xi$'];
t= title(naam,'interpreter','latex');
title_FontSize = 36;
t.FontSize = title_FontSize;
l = xlabel('samples','interpreter','latex');
l.FontSize = label_FontSize;
l= ylabel('meters','interpreter','latex');
%set(gca,'FontSize',20)
l.FontSize = label_FontSize;

l = legend('Error $\xi$', 'End PV array line', 'Start PV array line', 'interpreter', 'latex');
l.FontSize = legend_FontSize;
ax = gca;
ax.FontSize =25;



mean_error_nav =mean( abs(err_y(2500:end)))
std_error_nav = std(abs( err_y(2500:end)))





folder = '/home/lucamora/PHD/paper_for_journal_publication/Test_data/Test_Predosa/Variable_from_matlab_ws/Only_RGB_vision_error';
load(folder)
RGB_error_from_vision_line = error_from_vision_line;
RGB_mean_error = mean_e;
RGB_std_error = std_e;



% Control Error e


% Control error e
% error_from_vision_line = [0*error_from_vision_line(1:494);  error_from_vision_line(495:end)];
% mean_error = error_from_vision_line;
% % mean_error = [error_from_vision_line_1; error_from_vision_line_2];
error_from_vision_line = RGB_error_from_vision_line(1:6158);
mean_e = mean(abs(RGB_error_from_vision_line(2499: end)));
std_e = std(abs(RGB_error_from_vision_line(2499: end)));
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





% Navigation Error =  0.2807 +-  0.2547
% Control Error =  0.1218 +- 0.1719





