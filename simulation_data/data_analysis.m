

% BOTH CAMERA
%---> Exp 6
%---> Exp 4
%---> Exp 3

% RGB CAMERA
%---> Exp 3
%---> Exp 4

%Thermal Only
%---> Exp 4


% Note:
% Nei test a voltaggio i valori della retta GPS tra waypoint 1 e 2 considerati come valore vero dello stato sono:
% a = -0.35
% c = 0.0
%
% Nei test a Bollate i valori della retta GPS tra waypoint 1 e 2 considerati come valore vero dello stato sono:
% a = 0.0
% c = 0.0
%
% Nei punti dove lo stato o gli altri valori presentanio un plato perfettamehgte piatto sono i punti nel quale il drone tornava dal punto P2 al
% punto P1, quindi continuava a essere sovrascritto sempre lo stesso valore.
%
% Lo stato veniva reinizializzato con i valori della retta GPS
%

close all
load_multiple_photo = 0;
title_FontSize = 38;
label_FontSize = 34;
legend_FontSize = 28;


if load_multiple_photo == 0
    %Thermo Experiment
    % Exp_4
    % a_est_GF(1084) = 0;
    
    
    % RGB Experiment
    % Exp_2
    % a_est_GF = a_est_GF(1:6200);
    %c_est_GF = c_est_GF(1:6200);
    
    % a_est_GF = a_est_GF(2500);
    % c_est_GF = c_est_GF(0:6342);
    %      c_est_GF = c_est_GF(1: 5140);
    % a = ones(1035,1) * c_est_GF(5140);
    % c_est_GF = [c_est_GF;a];
    %
    
    %Both --> Exp_5
    %a_est(1885) = 0;
    
    
    
    %folder = '/home/lucamora/catkin_ws/Voltaggio_experiment/test_22_04_2021/telemetry_data/Exp_7';
    %folder = '/home/lucamora/Desktop/Test_Predosa/Exp_2';
    %folder = '/home/lucamora/PHD/paper_for_journal_publication/Test_data/Test_simulazione_Paper/Both_camera/Exp_12';
    %save
    % Leggere su qaderno rosso che cartella caricare per graifcare plot
    %folder = '/home/lucamora/PHD/paper_for_journal_publication/Test_data/test_GPS_coo_rototraslated/only_rotation/Exp_3';
    folder = '/home/lucamora/PHD/paper_for_journal_publication/Test_data/test_GPS_coo_rototraslated/roto_traslation/Exp_7';
    
    %folder = '/home/lucamora/Desktop/Navigation_in_critical_conditions/without_optimization/Exp_1';
    %folder =  '/home/lucamora/Desktop/Navigation_in_critical_conditions/with_optimization/Exp_2';
    %folder = '/home/lucamora/catkin_ws/simulation_data/Test_Navigation_in_critical_condition/Navigation_in_critical_conditions/with_optimization/test_2';
    
    only_rotation = 2;  %1 se only_rotation , 2 se rototraslation, 4 normal
    
    
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
    
    
    % Real Observation in BF
    fileID = fopen(strcat(folder ,'/a_obs_BF.txt'),'r');
    formatSpec = '%f';
    a_obs_BF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    % fileID = fopen(strcat(folder ,'/c_obs_BF.txt'),'r');
    % formatSpec = '%f';
    % c_obs_BF = fscanf(fileID,formatSpec);
    % fclose(fileID);
    
    
    % Observation estimated in BF
    fileID = fopen(strcat(folder ,'/a_obs_est_BF.txt'),'r');
    formatSpec = '%f';
    a_obs_est_BF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    fileID = fopen(strcat(folder ,'/c_obs_est_BF.txt'),'r');
    formatSpec = '%f';
    c_obs_est_BF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    % RGB aObservation in GF
    fileID = fopen(strcat(folder ,'/a_obs_RGB_GF.txt'),'r');
    formatSpec = '%f';
    a_obs_RGB_GF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/c_obs_RGB_GF.txt'),'r');
    formatSpec = '%f';
    c_obs_RGB_GF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    % Thermo aObservation in GF
    fileID = fopen(strcat(folder ,'/a_obs_thermo_GF.txt'),'r');
    formatSpec = '%f';
    a_obs_thermo_GF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    fileID = fopen(strcat(folder ,'/c_obs_thermo_GF.txt'),'r');
    formatSpec = '%f';
    c_obs_thermo_GF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    % Controlo D point in BF
    fileID = fopen(strcat(folder ,'/control_x_coo.txt'),'r');
    formatSpec = '%f';
    control_x_coo = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/control_y_coo.txt'),'r');
    formatSpec = '%f';
    control_y_coo = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    %Target Setpoint in BF
    fileID = fopen(strcat(folder ,'/X_target_setpoint_BF.txt'),'r');
    formatSpec = '%f';
    X_target_setpoint_BF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/Y_target_setpoint_BF.txt'),'r');
    formatSpec = '%f';
    Y_target_setpoint_BF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    %Target Setpoint in GF
    fileID = fopen(strcat(folder ,'/X_target_setpoint_GF.txt'),'r');
    formatSpec = '%f';
    X_target_setpoint_GF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/Y_target_setpoint_GF.txt'),'r');
    formatSpec = '%f';
    Y_target_setpoint_GF = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    
    
    % Desired velocity in GF (frame HOME)
    fileID = fopen(strcat(folder ,'/des_x_vel.txt'),'r');
    formatSpec = '%f';
    des_x_vel = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/des_y_vel.txt'),'r');
    formatSpec = '%f';
    des_y_vel = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    %Actual Drone Velocity in Frame HOME
    fileID = fopen(strcat(folder ,'/x_vel.txt'),'r');
    formatSpec = '%f';
    x_vel = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/y_vel.txt'),'r');
    formatSpec = '%f';
    y_vel = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/vel_z.txt'),'r');
    formatSpec = '%f';
    vel_z = fscanf(fileID,formatSpec);
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
    
    
    fileID = fopen(strcat(folder ,'/Yaw.txt'),'r');
    formatSpec = '%f';
    yaw = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/Yaw_des.txt'),'r');
    formatSpec = '%f';
    yaw_des = fscanf(fileID,formatSpec);
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
    
    
    
    %Drone Current Latitude
    fileID = fopen(strcat(folder ,'/drone_current_latitude.txt'),'r');
    formatSpec = '%f';
    drone_latitude = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    %Drone Current Latitude
    fileID = fopen(strcat(folder ,'/drone_current_longitude.txt'),'r');
    formatSpec = '%f';
    drone_longitude = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    %Offset From Vision LINE IN DJI
    fileID = fopen(strcat(folder ,'/ultrasonic_offset.txt'),'r');
    formatSpec = '%f';
    ultrasonic_offset = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    
    %Offset From Vision LINE IN DJI
    fileID = fopen(strcat(folder ,'/z_pixel_offset.txt'),'r');
    formatSpec = '%f';
    z_pixel_offset = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    %Offset From Vision LINE IN DJI
    fileID = fopen(strcat(folder ,'/z_pos_cmd.txt'),'r');
    formatSpec = '%f';
    z_pos_cmd = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    %Offset From Vision LINE IN DJI
    fileID = fopen(strcat(folder ,'/z_vel_des.txt'),'r');
    formatSpec = '%f';
    z_vel_des = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    %Offset from Array line in Gazebo
    % fileID = fopen(strcat(folder ,'/y_offset.txt'),'r');
    % formatSpec = '%f';
    % y_offset = fscanf(fileID,formatSpec);
    % fclose(fileID);
    
    
    
    
    a_line_GF = zeros(length(a_est_GF),1);
    c_line_GF = zeros(length(c_est_GF),1);
    
    a_line_GF = -0.36+a_line_GF;
    
    
    
    
    % %Evaluate error between the drone position and the òine estimated
    % %GPS coordinates of the two waypoints
    % waypoints_lat = [44.611266,44.611378];
    % waypoints_lon =  [8.860897,  8.860580];
    % C_EARTH = 6378137;
    %
    % wayp_dist_take_off_local_pos = [x_pos(1065), y_pos(1065)];
    %
    % %Trovo in GPS coo il punto che corrisponde a x_pos(1065), y_pos(1065) da
    % %local position
    % delta_y = x_pos(1065);
    % delta_x = y_pos(1065);
    %
    % delta_lat = delta_x/(C_EARTH * pi/180);
    % delta_lon = delta_y/(C_EARTH * pi/180 * cos( pi/180* 44.611266));
    %
    % mission_start_lat = 44.611266- delta_lat;
    % mission_start_lon = 8.860897 - delta_lon;
    %
    % %Trovo la distanza in local_pos offset tra il waypoint di start e il punto
    % %P2. Considero il frame local pos fissato n elle coordinate di
    % %mission_start.
    %
    % delta_lat_nav = waypoints_lat(2) - mission_start_lat;
    % delta_lon_nav = waypoints_lon(2) - mission_start_lon;
    %
    % delta_x = delta_lat_nav * C_EARTH * pi/180;
    % delta_y = delta_lon_nav  * C_EARTH * pi/180 * cos(pi/180 *  waypoints_lat(2));
    %
    %
    
    %Considero offset tra Punto di Take Off e punto P1 dopo Joystic di inizio
    %missione
    % delta_x = x_pos(2000);
    % delta_y = y_pos(2000);
    %
    % %Calcolo offset tra punto start e resto dei punti sulla navigazione
    % for ii = 1 : length(x_pos)
    %     new_x_pos = x_pos - delta_x;
    %     new_y_pos = y_pos - delta_y;
    % end
    
    %considero angolo di yaw tra pu to di start e di end
    
    
    % theta = atan2(new_y_pos(3580),new_x_pos(3580));
    %
    %
    % %Calcolo distanza punto retta tra frame drone e retta stimata
    % start_nav_new_x_pos = x_pos(2000);
    % start_nav_new_y_pos = y_pos(2000);
    %
    % for ii = 1 : length(x_pos(2000:end-1))
    %     ii_ = 2000 + ii;
    %     %Calcolo equazione retta stimata
    %     e(ii) = (abs(-1*a_est_GF(ii_)*new_x_pos(ii_) + new_y_pos(ii_) + -1*c_est_GF(ii_))/sqrt(a_est_GF(ii_)^2 + 1));
    % end
    
    
    
    
    %
    % figure
    % p = plot(new_x_pos);
    % p.LineWidth = 1.5;
    % hold on
    % p = plot(new_y_pos)
    % p.LineWidth = 1.5;
    % title('New UAV 2D positions')
    % xlabel('sample')
    % ylabel('Value')
    %
    
    
    % error_from_vision_line = error_from_vision_line;
    % error_from_vision_line_1 = error_from_vision_line(224:1171);
    % error_from_vision_line_2 = error_from_vision_line(1301:end);
    %error_from_vision_line =  error_from_vision_line(1087:end);
    %error_from_vision_line = error_from_vision_line(2433:6159);
    % error_from_vision_line_1 = error_from_vision_line(1:1170);
    % error_from_vision_line_2 = error_from_vision_line(1302:end);
%     for ii = 1 : length(error_from_vision_line)
%         if (control_y_coo(ii) > 0)
%             error_from_vision_line(ii) = -1*error_from_vision_line(ii);
%         end
%     end
    %error_from_vision_line = error_from_vision_line(1:6347);
    mean_error = error_from_vision_line;
    % mean_error = [error_from_vision_line_1; error_from_vision_line_2];
    mean_e = mean(mean_error);
    std_e = std(mean_error);
    figure
    p = plot(error_from_vision_line);
    p.LineWidth = 1.5;
    naam = ['Error from Estimated Vision Line'];
    t = title(naam,'interpreter', 'latex');
    t.FontSize = title_FontSize;
    l = xlabel('sample', 'interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = ylabel('meters','interpreter', 'latex');
    l.FontSize = label_FontSize;
    ax = gca;
    ax.FontSize =25;
    
    
    % l = legend('Error from Vision Line');
    % l.FontSize = legend_FontSize;
    
  % save('/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Only_RGB_vision_error.mat','error_from_vision_line', 'mean_e', 'std_e');
    
    
    if only_rotation == 2
        a_est_GF = [a_est_GF(1: 690); a_est_GF(690 + 165: end)]
        c_est_GF = [c_est_GF(1: 690); c_est_GF(690 + 165: end)]
        
    end
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
    
    naam = ['Estimation of line parameter $a^W$'];
    t = title(naam,'interpreter', 'latex');
    t.FontSize = title_FontSize;
    l = xlabel('samples', 'interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = ylabel('value','interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = legend('a Estimated', 'interpreter', 'latex');
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
    % p = plot(c_line_GF)
    % p.LineWidth = 1.5
    
    naam = ['Estimation of line parameter $c^W$'];
    t = title(naam,'interpreter', 'latex');
    t.FontSize = title_FontSize;
    l = xlabel('samples', 'interpreter', 'latex');
    
    l.FontSize = label_FontSize;
    l = ylabel('meters','interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = legend('c Estimated', 'interpreter', 'latex');
    l.FontSize = legend_FontSize;
    ax = gca;
    ax.FontSize =25;
    
    
    
    
    figure
    p = plot(a_obs_BF);
    p.LineWidth = 1.5;
    title('Observation in BF -- Angular Coefficient a of the line')
    xlabel('sample')
    ylabel('Value')
    legend('a Observed ')
    
    
    % figure
    % p = plot(c_obs_BF);
    % p.LineWidth = 1.5;
    % title('Observation in BF --  intersection with Y axis')
    % xlabel('sample')
    % ylabel('meter')
    % legend('c Observed ')
    
    
    
    figure
    p = plot(a_obs_est_BF);
    p.LineWidth = 1.5;
    title('Observation Estimated in BF -- Angular Coefficient a of the line')
    xlabel('sample')
    ylabel('Value')
    legend('a Observation Estimation')
    
    
    figure
    p = plot(c_obs_est_BF);
    p.LineWidth = 1.5;
    title('Observation Estimated  in BF --  intersection with Y axis')
    xlabel('sample')
    ylabel('meters')
    legend('c Observation Estimation')
    
    
    
    
    figure
    p = plot(a_obs_RGB_GF);
    p.LineWidth = 1.5;
    title('Observation in GF --  Angular Coefficient a of the line')
    xlabel('sample')
    ylabel('value')
    legend('a Observation GF')
    
    
    
    figure
    p = plot(c_obs_RGB_GF);
    p.LineWidth = 1.5;
    title('Observation in GF -- intersection with Y axis')
    xlabel('sample')
    ylabel('value')
    legend('c Observation GF')
    
    
    
    figure
    p = plot(a_obs_thermo_GF);
    p.LineWidth = 1.5;
    title('Observation in GF -- Angular Coefficient a of the line')
    xlabel('sample')
    ylabel('value')
    legend('a Observation GF')
    
    
    figure
    p = plot(c_obs_thermo_GF);
    p.LineWidth = 1.5;
    title('Observation in GF -- intersection with Y axis')
    xlabel('sample')
    ylabel('meters')
    legend('c Observation GF')
    
    
    
    
    
    
    
    
    
    
    
    
    
    figure
    p = plot(yaw);
    p.LineWidth = 1.5;
    hold on
    p = plot(yaw_des);
    p.LineWidth = 1.5;
    title('Yaw')
    xlabel('sample')
    ylabel('rad')
    legend('Yaw', 'Yaw des')
    
    
    
    
    
    
    
    figure
    % control_x_coo = control_x_coo(1: 6159);
    % control_y_coo = control_y_coo(1: 6159);
    %
    p = plot(control_x_coo);
    p.LineWidth = 1.5;
    hold on
    p = plot(control_y_coo);
    p.LineWidth = 1.5;
    legend('X coo BF', 'Y coo BF')
    naam = ['Control Point in BF'];
    t = title(naam,'interpreter', 'latex');
    t.FontSize = title_FontSize;
    l = xlabel('samples', 'interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = ylabel('meters','interpreter', 'latex');
    l.FontSize = label_FontSize;
    l = legend('X control point in BF','Y control point in BF', 'interpreter', 'latex');
    l.FontSize = legend_FontSize;
    
    
    
    
    
    figure
    p = plot(x_pos);
    p.LineWidth = 1.5;
    hold on
    p = plot(y_pos);
    p.LineWidth = 1.5;
    title('UAV Position in GF')
    xlabel('sample')
    ylabel('meters')
    legend('X GF', 'Y GF')
    
    figure
    p = plot(x_pos, y_pos);
    p.LineWidth = 1.5;
    title('UAV 2D Position in GF')
    xlabel('sample')
    ylabel('meters')
    
    
    % figure
    % p = plot(y_offset);
    % p.LineWidth = 1.5;
    % title('Offset From Array Line')
    % xlabel('sample')
    % ylabel('meter')
    
    
    figure
    p = plot(error_from_GPS_line);
    p.LineWidth = 1.5;
    title('Error From GPS LINE')
    xlabel('sample')
    ylabel('meters')
    
    
    % for ii = 1: length(des_x_vel)
    %     if (des_x_vel(ii) > 1 ||  des_x_vel(ii) < -1)
    %         des_x_vel(ii) = 0;
    %     end
    %
    %     if (des_y_vel(ii) > 1 ||  des_y_vel(ii) < -1)
    %         des_y_vel(ii) = 0;
    %     end
    %
    % end
    
    
    
    
    fileID = fopen(strcat(folder ,'/x_vel.txt'),'r');
    formatSpec = '%f';
    x_vel = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    fileID = fopen(strcat(folder ,'/y_vel.txt'),'r');
    formatSpec = '%f';
    y_vel = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    
    figure
    p = plot(x_vel);
    p.LineWidth = 1.5;
    hold on
    p = plot(y_vel);
    p.LineWidth = 1.5;
    title('Desired Velocities')
    xlabel('sample')
    ylabel('meters/sec')
    legend('X vel des ', 'Y vel_des')
    
    
    
    
    figure
    p = plot(des_x_vel);
    p.LineWidth = 1.5;
    hold on
    p = plot(des_y_vel);
    p.LineWidth = 1.5;
    title('Desired Velocities')
    xlabel('sample')
    ylabel('meters/sec')
    legend('X vel des ', 'Y vel_des')
    
    
    
    figure
    p = plot(drone_latitude, drone_longitude);
    p.LineWidth = 1.5;
    
    title('UAV latitude and longitude 2D')
    xlabel('sample')
    ylabel('meters')
    legend('UAV GPS location')
    
    
    figure
    p = plot(ultrasonic_offset);
    p.LineWidth = 1.5;
    title('Ultrasonic offset')
    xlabel('sample')
    ylabel('meters')
    
    
    figure
    p = plot(z_pixel_offset);
    p.LineWidth = 1.5;
    title('Z Pixel Offset')
    xlabel('sample')
    ylabel('meters')
    
    
    figure
    p = plot(z_pos_cmd);
    p.LineWidth = 1.5;
    title('Z Position Cmd')
    xlabel('sample')
    ylabel('meters')
    
    
    figure
    p = plot(z_vel_des);
    p.LineWidth = 1.5;
    title('Z Desired Velocity')
    xlabel('sample')
    ylabel('meters/sec')
    
    
    
    %Converting Drone latitude and longitude in local position relatively to
    %the start and end of each solar array
    
    % start_P1_value = 246;
    % end_P1_value = 1037;
    %
    % start_P2_value = 1312;
    % end_P2_value = 2093;
    %
    %
    % origin_lat = drone_latitude(2);
    % origin_lon = drone_longitude(2);
    %
    % start_P1_latitude = drone_latitude(start_P1_value);
    % start_P1_longitude = drone_longitude(start_P1_value);
    %
    % end_P1_latitude =  drone_latitude(end_P1_value);
    % end_P1_longitude =  drone_longitude(end_P1_value);
    %
    %
    % start_P2_latitude = drone_latitude(start_P2_value);
    % start_P2_longitude = drone_longitude(start_P2_value);
    %
    % end_P2_latitude =  drone_latitude(end_P2_value);
    % end_P2_longitude =  drone_longitude(end_P2_value);
    %
    % C_EARTH = 6378137;
    % deg2rad = pi/180;
    %
    % P1_start_Y = (start_P1_latitude - origin_lat) * deg2rad * C_EARTH;
    % P1_start_X = (start_P1_longitude - origin_lon) * deg2rad * C_EARTH *  cos(deg2rad*start_P1_latitude );
    %
    %
    % P1_end_Y = (end_P1_latitude - origin_lat) * deg2rad * C_EARTH;
    % P1_end_X = (end_P1_longitude - origin_lon) * deg2rad * C_EARTH *  cos(deg2rad*end_P1_latitude );
    %
    %
    % P2_start_Y = (start_P2_latitude - origin_lat) * deg2rad * C_EARTH;
    % P2_start_X = (start_P2_longitude - origin_lon) * deg2rad * C_EARTH *  cos(deg2rad*start_P2_latitude );
    %
    %
    % P2_end_Y = (end_P2_latitude - origin_lat) * deg2rad * C_EARTH;
    % P2_end_X = (end_P2_longitude - origin_lon) * deg2rad * C_EARTH *  cos(deg2rad*end_P2_latitude );
    %
    % %Connect point P1 Start con P1 end
    % diff1 = end_P1_value - start_P1_value;
    % diff2 = end_P2_value - start_P2_value;
    % P1_start_X = -3.60;
    % P1_start_Y = 5.40;
    % P1_end_X = -75.71;
    % P1_end_Y = 5.40;
    %
    % P2_start_X = -75.71;
    % P2_start_Y = 10.0400;
    % P2_end_X = -3.60;
    % P2_end_Y = 10.0400;
    %
    % %ROtate GPS Waypoints (only in simulation)
    % diff1_x = abs(P1_start_X - P1_end_X);
    % diff1_y = abs(P1_start_Y - P1_end_Y);
    %
    % theta = 0.01;
    % P1_end_X_new = cos(theta)*diff1_x - sin(theta)*diff1_y - P1_start_X;
    % P1_end_Y_new = sin(theta)*diff1_x + cos(theta)*diff1_y + P1_start_Y;
    %
    %
    % diff1_x = abs(P1_start_X - P2_start_X);
    % diff1_y = abs(P1_start_Y - P2_start_Y);
    %
    % theta = 0.01;
    % P2_start_X_new = cos(theta)*diff1_x - sin(theta)*diff1_y - P1_start_X;
    % P2_start_Y_new = sin(theta)*diff1_x + cos(theta)*diff1_y + P1_start_Y;
    %
    %
    % diff1_x = abs(P1_start_X - P2_end_X);
    % diff1_y = abs(P1_start_Y - P2_end_Y);
    %
    % theta = 0.01;
    % P2_end_X_new = cos(theta)*diff1_x - sin(theta)*diff1_y + P1_start_X;
    % P2_end_Y_new = sin(theta)*diff1_x + cos(theta)*diff1_y + P1_start_Y;
    %
    %
    %
    % Gps_path_P1_start_end_X = [P1_start_X, P1_end_X];
    % Gps_path_P1_start_end_Y = [P1_start_Y , P1_end_Y];
    %
    % Gps_path_P2_start_end_X = [P2_start_X, P2_end_X];
    % Gps_path_P2_start_end_Y = [ P2_start_Y, P2_end_Y];
    %
    %
    % Gps_path_P1_start_end_X = [P1_start_X, -1*P1_end_X_new];
    % Gps_path_P1_start_end_Y = [P1_start_Y , P1_end_Y_new];
    %
    % Gps_path_P2_start_end_X = [-1*P2_start_X_new, P2_end_X_new];
    % Gps_path_P2_start_end_Y = [ P2_start_Y_new, P2_end_Y_new];
    %
    %
    % % r = 2.0;
    % % th = 0:pi/50:2*pi;
    % % counter = 1;
    % % for ii = 1 : length(y_pos)
    % %
    % %     if (y_pos(ii) > 12.9)
    % %         index_array(counter) = ii;
    % %         counter = counter + 1;
    % %     end
    % %
    % % end
    % %
    % % y_pos = y_pos(1 : index_array(1));
    % % x_pos = x_pos(1 : index_array(1));
    % % x_pos = x_pos(1:6200);
    % % y_pos = y_pos(1:6200);
    %
    %
    % %Rotate the points in order to align the graph
    % point1 = 1169;
    % point2 = length(y_pos);
    %
    % d_x = x_pos(point2) - x_pos(point1);
    % d_y = y_pos(point2) - y_pos(point1);
    %
    % i = sqrt(d_x^2 + d_y^2);
    %
    % theta = acos(d_x/i)
    %
    % value = point2 - point1;
    %
    % % Evaluate all the points respect point 1
    % for ii = 1 : value
    %     it = point1 + ii;
    %     x_new(ii) = x_pos(it) - x_pos(point1);
    %     y_new(ii) = y_pos(it) - y_pos(point1);
    % end
    %
    % for ii = 1 : value
    %     x_new_rotated(ii) = x_new(ii)*cos(-theta) -  y_new(ii)*sin(-theta) + x_pos(point1);
    %     y_new_rotated(ii) =  x_new(ii)*sin(-theta) +  y_new(ii)*cos(-theta) + y_pos(point1);
    % end
    %
    %
    % x_pos = [x_pos(1:point1); x_new_rotated'];
    % y_pos = [y_pos(1:point1); y_new_rotated'];
    %
    % GPS_path_x = [Gps_path_P1_start_end_X ,Gps_path_P2_start_end_X];
    % GPS_path_y = [Gps_path_P1_start_end_Y, Gps_path_P2_start_end_Y];
    
    if only_rotation == 1
        %% ADD ROTOTRASLATED WAYPOINTS POSITIONS
        % ------> ONLY ROTATION
        %----> Scommentare questa parte solo se si sta graficando folder = '/home/lucamora/catkin_ws/simulation_data/test_GPS_coo_rototraslated/only_rotation/Exp_3';
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
        P1_start = [vpa(-26.5367453628); vpa(10.9530490274)];
        P1_end = [vpa(56.7886758395902); vpa(10.9530490274)];
        %P1_end = [vpa(-16.7886758395902); vpa(10.056757384)];
        
        P2_start = [vpa(56.8785473703139); vpa(15.9328472193)];
        %P2_start = [vpa(-16.8785473703139); vpa(15.9328472193)];
        P2_end = [vpa(-28.48304882148); vpa(15.9328472193)];
        
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
        
         
        
        % i_x = find(x_pos == 57.677);
        % i_y = y_pos(i_x);
        %
        % x_pos_to_save = x_pos(i_x : end);
        % y_pos_to_save = y_pos(i_x : end);
        % %
        % save('path.mat','x_pos_to_save','y_pos_to_save')
        
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
        
        hold on
        p1 = plot(GPS_rotated_path_x, GPS_rotated_path_y );
        p1.Marker = 'square';
        p1.LineStyle = '-.';
        p1.LineWidth = 2;
        p1.MarkerSize = 20;
        p1.MarkerEdgeColor = 'r';
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
        naam = ['UAV 2D Position in W'];
        t= title(naam,'interpreter','latex');
        title_FontSize = 36;
        t.FontSize = title_FontSize;
        l = xlabel('meters','interpreter','latex');
        l.FontSize = label_FontSize;
        l= ylabel('meters','interpreter','latex');
        %set(gca,'FontSize',20)
        l.FontSize = label_FontSize;
        l = legend('UAV Path','Real Path', 'GPS Waypoints', 'interpreter','latex');
        l.FontSize = legend_FontSize;
        ax = gca;
        ax.FontSize =25;
        axis equal
        
        save('only_rot_path_plot','x_pos', 'y_pos', 'GPS_true_path_x' , 'GPS_true_path_y' , 'GPS_rotated_path_x' , 'GPS_rotated_path_y' );
        % Evaluate error between real path and UAV trajectory
        a = find(x_pos == -27.7069);
        b = find(x_pos == 55.9631);
        c = find(x_pos == 55.921);
        d = find(x_pos == -26.2222);
        
        err_ab = GPS_true_path_y(1) - y_pos(a:b);
        err_cd = GPS_true_path_y(3) - y_pos(c:d);
        err_ab_mean = mean(err_ab);
        err_cd_mean = mean(err_cd);
        std_err_ab = std(err_ab);
        std_err_cd = std(err_cd);
    elseif (only_rotation == 2)
            
            
            
            %True Waypoints Position
            P1_start = [-26.5367453628; 10.9530490274];
            P1_end = [56.7886758395902; 10.964892014];
            %P1_end = [vpa(-16.7886758395902); vpa(10.056757384)];
            
            P2_start = [56.8785473703139; 15.9328472193];
            %P2_start = [vpa(-16.8785473703139); vpa(15.9328472193)];
            P2_end = [-28.48304882148; 15.91789382012];
            
            
            
            GPS_true_path_x = [P1_start(1), P1_end(1), P2_start(1), P2_end(1)];
            GPS_true_path_y = [P1_start(2), P1_end(2), P2_start(2), P2_end(2)];
            
            %ROTOTRASLATED GPS ---> ONLY ROTATION EXP3
            
            P = [P1_start, P1_end, P2_start, P2_end];
            % rotate local coordinates
            
            
            % Place rotational frame in P1 start
            P = P - P1_start;
            tx = 1.0; %1.0;
            ty = 1.27; %0.8
            gamma = 0.1; % rad
            
            R_z = [cos(gamma) -sin(gamma) tx ;...
                sin(gamma) cos(gamma) ty];
            
            
            
            
            for ii = 1 : 4
                
                P_new(:,ii) = R_z * [P(:,ii);1];
                
                P_new(1, ii) = P_new(1, ii) + P1_start(1)
                P_new(2, ii) = P_new(2, ii) + P1_start(2)
                
            end
            %
            % %
            % P_new(1,3)  = P1_start(1) + P_new(1,3);
            % P_new(1,4) = P1_start(1) + P_new(1,4);
            % P_new(2,3) = P1_start(1)
            GPS_rotated_path_x = [P_new(1,1), P_new(1,2), P_new(1,3), P_new(1,4)];
            GPS_rotated_path_y = [P_new(2,1), P_new(2,2), P_new(2,3), P_new(2,4)];
            
            
            
            x_pos_new = x_pos(166:end);
            y_pos_new = y_pos(166:end);
            
            x_pos = x_pos_new;
            y_pos = y_pos_new;
            
            x_pos = x_pos - x_pos(1);
            
            
            x_pos_new  = x_pos +   P1_start(1);
            y_pos_new = y_pos + P_new(2,1) -(y_pos(1)) ;
            
            x_pos = x_pos_new;
            y_pos = y_pos_new;
%             
   %ROtate middle part of the navigation
            
            x_ref = 1078;
            y_ref = 1078;
            
           
            for ii = 1:length(x_pos(x_ref:end-1))
                x_new(:,ii) = (x_pos(x_ref) - x_pos(x_ref + ii)) * cos(0.05) - (y_pos(x_ref + ii) - y_pos(x_ref + ii)) * sin(0.05);
                y_new(:,ii) = (x_pos(x_ref) - x_pos(x_ref + ii)) * sin(0.05) + (y_pos(x_ref) - y_pos(x_ref + ii)) * cos(0.05);
                
                x_new(:,ii) = x_pos(x_ref) -  x_new(:,ii);
                y_new(:,ii) =  y_pos(x_ref) -  y_new(:,ii);
            end
            
            x_pos = [x_pos(1:x_ref); x_new'];
            y_pos = [y_pos(1:y_ref); y_new'];
            
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
            
            hold on
            p1 = plot(GPS_rotated_path_x, GPS_rotated_path_y );
            p1.Marker = 'square';
            p1.LineStyle = '-.';
            p1.LineWidth = 2;
            p1.MarkerSize = 20;
            p1.MarkerEdgeColor = 'r';
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
            naam = ['UAV 2D Position in GF'];
            t= title(naam,'interpreter','latex');
            title_FontSize = 36;
            t.FontSize = title_FontSize;
            l = xlabel('meters','interpreter','latex');
            l.FontSize = label_FontSize;
            l= ylabel('meters','interpreter','latex');
            %set(gca,'FontSize',20)
            l.FontSize = label_FontSize;
            l = legend('UAV Path','Real Path', 'GPS Waypoints', 'interpreter','latex');
            l.FontSize = legend_FontSize;
            ax = gca;
            ax.FontSize =25;
            axis equal 
            
            %save('roto_trasl_path_plot','x_pos', 'y_pos', 'GPS_true_path_x' , 'GPS_true_path_y' , 'GPS_rotated_path_x' , 'GPS_rotated_path_y' );
            %
           
            
            
            %Graficate the navigation error 
            % Error distance from true line 
            
            
            err_y_P1_start_P1_end = P1_start(2) - y_pos(1:length(y_pos)/2);
            
            %y_pos_err_2 = [y_pos(1344:end)];
            err_y_P2_start_P2_end = P2_start(2) - y_pos(length(y_pos)/2 + 1: end);
            
            err_y = [err_y_P1_start_P1_end; err_y_P2_start_P2_end];
            
          
            mean_err = mean(err_y);
            std_err_y = std(err_y); 
            
%             err_y(858:1085) = err_y(857);
            line1_s_y = [-2.0; 2];
            line1_s_x = [575; 575];
            line2_s_y = [-2.0; 2];
            line2_s_x = [783; 783];
            
            
            
            figure
            p = plot(err_y);
            p.LineWidth = 1.5;
            hold on 
            p = plot(line_s_x, line_s_y);
            p.LineWidth = 1.5;
            p.LineStyle = '-.';
            
            hold on 
            p = plot(line2_s_x, line2_s_y);
            p.LineWidth = 1.5;
            p.LineStyle = '-.';
            
            
             % Evaluate error between real path and UAV trajectory
%             i1 = 624;
%             i2 = 870;
            
            y_pos = double(y_pos);
            err_ab = GPS_true_path_y(1) - y_pos(1:i1);
            err_cd = GPS_true_path_y(3) - y_pos(i2:end);
            err_ab_mean = mean(err_ab);
            err_cd_mean = mean(err_cd);
            std_err_ab = std(err_ab);
            std_err_cd = std(err_cd);
            
        else
            
            % Seziine relativa al percorso senza rotaziini dei gps
            % waypoints 
            
            
            
            
            %True Waypoints Position
            P1_start = [-26.5367453628; 10.9530490274 - 0.50];
            P1_end = [57.7886758395902; 10.9530490274 - 0.50];
            %P1_end = [vpa(-16.7886758395902); vpa(10.056757384)];
            
            P2_start = [56.8785473703139; 15.9328472193 + 0.30];
            %P2_start = [vpa(-16.8785473703139); vpa(15.9328472193)];
            P2_end = [-28.48304882148; 15.9328472193 + 0.30];
            
            
            
            GPS_true_path_x = [P1_start(1), P1_end(1), P2_start(1), P2_end(1)];
            GPS_true_path_y = [P1_start(2), P1_end(2), P2_start(2), P2_end(2)];
            
            %Rotate all the point between P1start and P1 end of .. degrees
            
            
            len = 1343 - 354;
            theta = 0.5*pi/180;
            x_pos_reframe = x_pos - x_pos(354);
            y_pos_reframe = y_pos - y_pos(354);
            for ii = 1 : len
                x_pos_rot(ii) = cos(theta) * x_pos_reframe(498 + ii) - sin(theta)*y_pos_reframe(498 + ii);
                y_pos_rot(ii) = sin(theta) * x_pos_reframe(498 + ii) + cos(theta)*y_pos_reframe(498 + ii);
            end
            x_pos_rotation = x_pos_rot +  x_pos(354);
            y_pos_rotation = y_pos_rot +  y_pos(354);
            x_pos = [x_pos(1:354); x_pos_rotation'; x_pos(1343 : end)];
            y_pos = [y_pos(1:354); y_pos_rotation'; y_pos(1343 : end)];
            
            y_pos_off = y_pos(1344 : end) + 0.8397;
            y_pos = [y_pos(1:354); y_pos_rotation'; y_pos_off];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           %ROTOTRASLATED GPS ---> ONLY ROTATION EXP3
%             
%             P = [P1_start, P1_end, P2_start, P2_end];
% 
%  
%             x_pos = x_pos(1599:end);
%             y_pos = y_pos(1599:end);
%              
%           
%             x_pos = -1*x_pos;
%             
%             
%             offsetx = x_pos(1) - P1_start(1);
%             offsety = y_pos(1) - P1_start(2);
%             
%             x_pos = x_pos - offsetx;
%             y_pos = y_pos - offsety;
%             
            
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % DECOMMENTARE PER GRAFICO RELATIVO A PERCORSO CON TEXTURE NON OTTIMIZZATA
 % RELATIVA A FIOLDER: WITHOUT_OPT/EXP_4
%             x_pos = x_pos(1502:end);
%             y_pos = y_pos(1502:end);
%              
%           
%               x_pos = -1*x_pos;
%             
%             
%             offsetx = x_pos(1) - P1_start(1);
%             offsety = y_pos(1) - P1_start(2);
%             
%             x_pos = x_pos - offsetx;
%             y_pos = y_pos - offsety;
%             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
           
            
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
            naam = ['UAV 2D Position in GF'];
            t= title(naam,'interpreter','latex');
            title_FontSize = 36;
            t.FontSize = title_FontSize;
            l = xlabel('meters','interpreter','latex');
            l.FontSize = label_FontSize;
            l= ylabel('meters','interpreter','latex');
            %set(gca,'FontSize',20)
            l.FontSize = label_FontSize;
            l = legend('UAV Path','Ideal Path', 'interpreter','latex');
            l.FontSize = legend_FontSize;
            ax = gca;
            ax.FontSize =25;
            
            
            % Error distance from true line 
            
            y_pos_err_1 = [y_pos(354:1343)];
            err_y_P1_start_P1_end =P1_start(2) - y_pos_err_1;
            
            y_pos_err_2 = [y_pos(1344:end)];
            err_y_P2_start_P2_end = P2_start(2) - y_pos_err_2;
            
            err_y = [err_y_P1_start_P1_end; err_y_P2_start_P2_end];
            
          
            mean_err = mean(err_y);
            std_err_y = std(err_y); 
            
            err_y(858:1085) = err_y(857);
            line_s_y = [-2.0; 2];
            line_s_x = [1085; 1085];
            
            figure
            p = plot(err_y);
            p.LineWidth = 1.5;
            hold on 
            p = plot(line_s_x, line_s_y);
            p.LineWidth = 1.5;
            p.LineStyle = '-.';
           
            
           
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
         
    
            ax = gca;
            ax.FontSize =25;
            
            
            
            
        end
        
        
        
        
        
        %save('/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Only_RGB_UAV_path.mat', 'x_pos', 'y_pos', 'Gps_path_P1_start_end_X', 'Gps_path_P1_start_end_Y');
    else
%         folder = '/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Only_Thermo_UAV_path';
%         load(folder)
%         
%         Thermo_GPS_path_P1_start_end_X = Gps_path_P1_start_end_X;
%         Thermo_GPS_path_P1_start_end_Y = Gps_path_P1_start_end_Y;
%         Thermo_x_pos = x_pos;
%         Thermo_y_pos = y_pos;
        
        folder = '/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Only_THERMO_vision_error';
        load(folder)
        Thermo_error_from_vision_line = error_from_vision_line;
        Thermo_mean_error = mean_e;
        Thermo_std_error = std_e;
        
%         folder = '/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Only_RGB_UAV_path';
%         load(folder)
%         
%         RGB_GPS_path_P1_start_end_X = Gps_path_P1_start_end_X;
%         RGB_GPS_path_P1_start_end_Y = Gps_path_P1_start_end_Y;
%         RGB_x_pos = x_pos;
%         RGB_y_pos = y_pos;
        
        folder = '/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Only_RGB_vision_error';
        load(folder)
        RGB_error_from_vision_line = error_from_vision_line;
        RGB_mean_error = mean_e;
        RGB_std_error = std_e;
        
        
%         folder = '/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Both_camera_UAV_path';
%         load(folder)
%         Both_GPS_path_P1_start_end_X = Gps_path_P1_start_end_X;
%         Both_GPS_path_P1_start_end_Y = Gps_path_P1_start_end_Y;
%         Both_GPS_path_P2_start_end_X = Gps_path_P2_start_end_X;
%         Both_GPS_path_P2_start_end_Y = Gps_path_P2_start_end_Y;
%         Both_x_pos = x_pos;
%         Both_y_pos = y_pos;
        
        folder = '/home/lucamora/Desktop/Test_Predosa/Variable_from_matlab_ws/Both_camera_vision_error';
        load(folder)
        Both_error_from_vision_line = error_from_vision_line;
        Both_mean_error = mean_e;
        Both_std_error = std_e;
        
%         %Path Plot
%         
%         figure
%         subplot(3, 1, 1)
%         p = plot(Thermo_x_pos, Thermo_y_pos);
%         p.LineWidth = 1.5;
%         hold on
%         p1 = plot(Thermo_GPS_path_P1_start_end_X, Thermo_GPS_path_P1_start_end_Y );
%         p1.Marker = 'square';
%         p1.LineStyle = '-.';
%         p1.MarkerSize = 20;
%         p1.MarkerEdgeColor = 'r';
%         p1.MarkerFaceColor = [0.5,0.5,0.5];
%         p1.LineWidth = 2;
%         
%         
%         naam = ['2D UAV path with Thermal camera'];
%         t = title(naam,'interpreter', 'latex');
%         t.FontSize = title_FontSize;
%         l = xlabel('meters', 'interpreter', 'latex');
%         l.FontSize = label_FontSize;
%         l = ylabel('meters','interpreter', 'latex');
%         l.FontSize = label_FontSize;
%         l = legend('UAV path', 'GPS Waypoints', 'interpreter', 'latex');
%         l.FontSize = 25;
%         ax = gca;
%         ax.FontSize =25;
%         
%         
%         subplot(3, 1, 2)
%         p = plot(RGB_x_pos, RGB_y_pos);
%         p.LineWidth = 1.5;
%         hold on
%         p1 = plot(RGB_GPS_path_P1_start_end_X, RGB_GPS_path_P1_start_end_Y );
%         p1.Marker = 'square';
%         p1.LineStyle = '-.';
%         p1.MarkerSize = 20;
%         p1.MarkerEdgeColor = 'r';
%         p1.MarkerFaceColor = [0.5,0.5,0.5];
%         p1.LineWidth = 2;
%         
%         
%         naam = ['2D UAV path with RGB camera'];
%         t = title(naam,'interpreter', 'latex');
%         t.FontSize = title_FontSize;
%         l = xlabel('meters', 'interpreter', 'latex');
%         l.FontSize = label_FontSize;
%         l = ylabel('meters','interpreter', 'latex');
%         l.FontSize = label_FontSize;
%         l = legend('UAV path', 'GPS Waypoints', 'interpreter', 'latex');
%         l.FontSize = 25;
%         ax = gca;
%         ax.FontSize =25;
%         
%         
%         GPS_path_x = [Both_GPS_path_P1_start_end_X, Both_GPS_path_P2_start_end_X];
%         GPS_path_y = [Both_GPS_path_P1_start_end_Y, Both_GPS_path_P2_start_end_Y];
%         
%         subplot(3, 1, 3)
%         p = plot(Both_x_pos, Both_y_pos);
%         p.LineWidth = 1.5;
%         hold on
%         p1 = plot(GPS_path_x, GPS_path_y );
%         p1.Marker = 'square';
%         p1.LineStyle = '-.';
%         p1.MarkerSize = 20;
%         p1.MarkerEdgeColor = 'r';
%         p1.MarkerFaceColor = [0.5,0.5,0.5];
%         p1.LineWidth = 2;
%         
%         
%         naam = ['2D UAV path with Both cameras'];
%         t = title(naam,'interpreter', 'latex');
%         t.FontSize = title_FontSize;
%         l = xlabel('meters', 'interpreter', 'latex');
%         l.FontSize = label_FontSize;
%         l = ylabel('meters','interpreter', 'latex');
%         l.FontSize = label_FontSize;
%         l = legend('UAV path', 'GPS Waypoints', 'interpreter', 'latex');
%         l.FontSize = 25;
%         ax = gca;
%         ax.FontSize =25;
        
        
        
        
        %Plot Error Vision
        
        figure
        subplot(3, 1, 1)
        p = plot(Thermo_error_from_vision_line);
        p.LineWidth = 1.5;
        mean_error_1 = mean(Thermo_error_from_vision_line(1091:end));
        Thermo_mean_error = mean(mean_error_1);
        std_error_1 = std(Thermo_error_from_vision_line(1091:end));
        Thermo_std_error = std_error_1;
        naam = ['Error from Estimated Vision Line'];
        
        %naam = ['Error from Vision Line with only Thermal Camera, mean $\mu = ', num2str(Thermo_mean_error), '$',' ,standard deviation $\sigma = ', num2str(Thermo_std_error), '$'];
        t = title(naam,'interpreter', 'latex');
        t.FontSize = title_FontSize;
        l = xlabel('samples', 'interpreter', 'latex');
        l.FontSize = label_FontSize;
        l = ylabel('meters','interpreter', 'latex');
        l.FontSize = label_FontSize;
        ax = gca;
        ax.FontSize =25;
        
        
        subplot(3, 1, 2)
        RGB_error_from_vision_line = RGB_error_from_vision_line(1: 6159);
        
        p = plot(RGB_error_from_vision_line);
        p.LineWidth = 1.5;
        mean_error_1 = mean(RGB_error_from_vision_line(2500:end));
        RGB_mean_error = mean(mean_error_1);
        std_error_1 = std(RGB_error_from_vision_line(2500:end));
        RGB_std_error = std_error_1;
        naam = ['Error from Estimated Vision Line'];
        
        %naam = ['Error from Vision Line with only RGB camera, mean $\mu = ', num2str(RGB_mean_error), '$',' ,standard deviation $\sigma = ', num2str(RGB_std_error), '$'];
        t = title(naam,'interpreter', 'latex');
        t.FontSize = title_FontSize;
        l = xlabel('samples', 'interpreter', 'latex');
        l.FontSize = label_FontSize;
        l = ylabel('meters','interpreter', 'latex');
        l.FontSize = label_FontSize;
        ax = gca;
        ax.FontSize =25;
        
        
        subplot(3, 1, 3)
        p = plot(Both_error_from_vision_line);
        
        %Non considero nel calcolo della media il salto di pannelli
        mean_error_1 = mean(Both_error_from_vision_line(1858:6090));
        mean_error_2  = mean(Both_error_from_vision_line(6894:end));
        mean_error = mean([mean_error_1; mean_error_2]);
        std_error_1 = std(Both_error_from_vision_line(1858:6090));
        std_error_2 = std(Both_error_from_vision_line(6894:end));
        std_error = std([std_error_1;std_error_2]);
        
        p.LineWidth = 1.5;
        naam = ['Error from Estimated Vision Line'];
        %naam = ['Error from Vision Line with Both cameras, mean $\mu = ', num2str(mean_error), '$',' ,standard deviation $\sigma = ', num2str(std_error), '$'];
        t = title(naam,'interpreter', 'latex');
        t.FontSize = title_FontSize;
        l = xlabel('samples', 'interpreter', 'latex');
        l.FontSize = label_FontSize;
        l = ylabel('meters','interpreter', 'latex');
        l.FontSize = label_FontSize;
        ax = gca;
        ax.FontSize =25;
        
        
        
        
        
        
        
    end
    
    
    
    
    
    
    
