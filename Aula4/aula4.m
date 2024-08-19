%% Clear and format
                                                                            % Close files and Clear Variables.            
clc; close all; clearvars; workspace;                                       % Make sure the workspace panel is showing.
fontSize=40; fontName='Impact'; fontColor=[0.15 0.22 0.33];                 % Format font size, style and color.     
format long g; format compact; fontSize2=32;                                % Long Numeric Values. Supress Blank Lines(Display).
addpath ../lib;

%% Ex List - (Uncomment name of exercise)
    
    List = {            
%         'Ex1 '
%         'Ex2 '
         'Ex3 '
        };
%==========================================================================        

if ismember('Ex1 ', List)
    %% Ex1 - Odometry calculation for differential drive:
    close all;
    
    % wheel diameter
    d = 0.20;
    % wheel separation
    D = 0.60;
    % gear ratio
    n = 1;
    % Pulses per turn
    Ce = 1024;

    % diff drive odometry equations
    k = (pi*d)/(n*Ce);
    
    % encoders measures:
    MR = [0,223,446,669,892,1115,1338,1561,1784,2007,2230,2453,2676,2899,3122,3345,3568,3791,4014,4237,4460,4683];
    ML = [0,120,240,360,480,600,720,840,960,810,750,690,630,570,510,450,621,873,1125,1377,1629,1881];
    
    % initial position:
    x = 0;
    y = 0;
    theta = 0; 
    dt = 0.2;  %tempo usado indiretamente nos encoders

    for n=2:numel(MR)
    
        % NL & NR from measures:
        NR = MR(n)-MR(n-1);
        NL = ML(n)-ML(n-1);
    
        % delta thetha, li
        d_Lr = k*NR;
        d_Ll = k*NL; 
        d_li = (d_Lr+d_Ll)/2;
        d_theta = (d_Lr-d_Ll)/D;
       
        
        % estimate final position and orientation:
        theta(n) = theta(n-1) + d_theta;
        x(n) = x(n-1) + d_li*cos(theta(n));
        y(n) = y(n-1) + d_li*sin(theta(n));
    
    end
    
    % plot positions
    plot(x,y,'.-b');
    x(n)
    y(n)
    theta(n)*180/pi





  

end

if ismember('Ex2 ', List)
    %% Ex2 - Odometry for tricycle
        close all;
    
    % wheel diameter
    d = 0.20;
    % distance to rear wheels
    L = 0.80;
    % gear ratio
    n = 1;
    % Pulses per turn
    Ce = 1024;
    % steering resolution
    res = 1;

    % diff drive odometry equations
    k = (pi*d)/(n*Ce);
    
    % encoders measures:
    S = [0,219,442,674,875,1113,1350,1520,1744,2001,2214,2440,2723,2934,3127,3399,3594,3860,4080,4321,4533,4701];
    alpha = [0,0,5,5,8,8,10,10,10,12,10,7,6,3,-4,-8,-10,-12,-14,-12,-10,-8];
    
    % initial position:
    x = 0;
    y = 0;
    theta = 0; 
    dt = 0.2;  %tempo usado indiretamente nos encoders

    for n=2:numel(S)
    
        % S & alpha from measures:
        NS = S(n)-S(n-1);
        alpha_rad = alpha(n)*pi/180;    %converter em radianos
    
        % delta li
        d_li = k*NS;
       
        % estimate final position and orientation:
        theta(n) = theta(n-1) + (d_li/L)*sin(alpha_rad);
        x(n) = x(n-1) + d_li*cos(alpha_rad)*cos(theta(n));
        y(n) = y(n-1) + d_li*cos(alpha_rad)*sin(theta(n));
    
    end
    
    % plot positions
    axis equal; grid on; hold on;
    plot(x,y,'.-b');
    x(n)
    y(n)
    theta(n)*180/pi 

    
end


if ismember('Ex3 ', List)
    %% Ex3 - Integrate data from smartphone for localization:
    close all;
    
    %try
    m = mobiledev
    %catch
    %end
% 
%     AA = [0 0 0];
%     WW = [0 0 0];
% 
%     while 1
%     a = m.Acceleration;
%     w = m.AngularVelocity;
%     AA = [AA; a];
%     WW = [WW; w];
%     plot(AA)
%     pause(0.1)
%     end
    
    %vel. angular
    Dt = 0.1;
    th = [0 0 0];
    while 1
        % Medir vel.angular inicial e final, e orientacao incrementa.
        wi = m.AngularVelocity;
        %wf = m.AngularVelocity;
        Dth = wi*Dt;
        th = th + Dth;
        th*180/pi; % verifica-se o drift
        pause(Dt)
    end


end
