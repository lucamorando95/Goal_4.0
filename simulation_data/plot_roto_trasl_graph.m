%% PLOT ROTOTRALSATED GPS COO GRAPH
close all
clear all

title_FontSize = 38;
label_FontSize = 34; 
legend_FontSize = 28;



% 
% load('only_rot_path_plot.mat')
% 
% 
%   figure
%   subplot(2,1,1)
%     p = plot(x_pos, y_pos);
%     p.LineWidth = 1.5;
%     hold on
%     p1 = plot(GPS_true_path_x, GPS_true_path_y );
%     p1.Marker = 'square';
%     p1.LineStyle = '-.';
%     p1.LineWidth = 2;
%     p1.MarkerSize = 20;
%     p1.MarkerEdgeColor = 'g';
%     p1.MarkerFaceColor = [0.5,0.5,0.5];
%     
%     hold on
%     p1 = plot(Gps_path_P2_start_end_X, Gps_path_P2_start_end_Y);
%     p1.Marker = 'square';
%     p1.LineStyle = '-.';
%     p1.MarkerSize = 20;
%     p1.MarkerEdgeColor = 'r';
%     p1.MarkerFaceColor = [0.5,0.5,0.5];
%     
%     hold on
%     p1 = plot(GPS_rotated_path_x, GPS_rotated_path_y );
%     p1.Marker = 'square';
%     p1.LineStyle = '-.';
%     p1.LineWidth = 2;
%     p1.MarkerSize = 20;
%     p1.MarkerEdgeColor = 'r';
%     p1.MarkerFaceColor = [0.5,0.5,0.5];
%     
%     hold on
%     p1 = plot(Gps_path_P2_start_end_X, Gps_path_P2_start_end_Y);
%     p1.Marker = 'square';
%     p1.LineStyle = '-.';
%     p1.MarkerSize = 20;
%     p1.MarkerEdgeColor = 'r';
%     p1.MarkerFaceColor = [0.5,0.5,0.5];
%     
%     hold on
%     plot(P2_start_X, P2_start_Y-9.5, 'sq', 'MarkerSize', 30);
%     hold on
%     plot(P2_end_X, P2_end_Y -9.5, 'sq', 'MarkerSize', 30);
%     
%     Hold on circle near waypoints
%     naam = ['UAV 2D Position in GF'];
%     t= title(naam,'interpreter','latex');
%     title_FontSize = 36;
%     t.FontSize = title_FontSize;
%     l = xlabel('meters','interpreter','latex');
%     l.FontSize = label_FontSize;
%     l= ylabel('meters','interpreter','latex');
%     set(gca,'FontSize',20)
%     l.FontSize = label_FontSize;
%     l = legend('UAV Path','Ideal Path', 'GPS Waypoints', 'interpreter','latex');
%     l.FontSize = legend_FontSize;
%     ax = gca;
%     ax.FontSize =25;
%     
    
    
    load('roto_trasl_path_plot.mat')

    
     subplot(2,1,2)
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
    l = legend('UAV Path','Ideal Path', 'GPS Waypoints', 'interpreter','latex');
    l.FontSize = legend_FontSize;
    ax = gca;
    ax.FontSize =25;
    
     
     