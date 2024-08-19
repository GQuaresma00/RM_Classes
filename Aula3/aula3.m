%% Clear and format
                                                                            % Close files and Clear Variables.            
clc; close all; clearvars; workspace;                                       % Make sure the workspace panel is showing.
fontSize=40; fontName='Impact'; fontColor=[0.15 0.22 0.33];                 % Format font size, style and color.     
format long g; format compact; fontSize2=32;                                % Long Numeric Values. Supress Blank Lines(Display).
addpath ../lib;

%% Ex List - (Uncomment name of exercise)
    
    List = {            
%         'Ex1 '
         'Ex2 '
        };
%==========================================================================        

if ismember('Ex1 ', List)
    %% Ex1 - Test function invkinDD():
    close all;
    
    %starting point (0,0,0)
    R = [ 0    0   0.2
          0.1 -0.1 0
         ];
     
    Rh = R; 
    Rh(3,:) = 1; %homogeneous version
    
    hR = fill(Rh(1,:), Rh(2,:), 'y');
    axis equal; grid on; hold on;

    %arrives at t=5s
    L = 0.5;
    t = 5;
    X = 2;
    %Y = 0;
    th = pi/6;   %mudar orientação e experimentar -pi/6, -pi/2
    %th = 170*pi/180;

    %use inverse kinematics to compare solution: Vr = 0.445; Vl = 0.393
    [VR,VL] = invkinDD(X,nan,th,L,t);
    disp([VR,VL])
    
    %Verify with direct kin  
    Dt = 0.1;  %interval step of simulation
    ST = t/Dt; %steps for simulation
    r = 0.1;   %wheel radius

    %empty arrays
    P = zeros(2,ST+1);
    th = zeros(1,ST+1);

    %starting point
    X0 = 0; 
    Y0 = 0;
    P0 = [X0;Y0];
    th0 = 0;

    %Init array
    P(:,1) = P0;
    th(1) = th0;

    %DD robot
    type = 1;

    %Ang vel
    w1 = VR/r;
    aw2 = VL/r;
    w3 = 0;

    for n=1:ST
        [Vx,Vy,w] = localvel(type,r,L,w1,aw2,w3);
        R = orm(th(n));
        V = inv(R)*[Vx;Vy;w];           %calcular CD
        P(:,n+1) = P(:,n)+V(1:2)*Dt;    %proximo ponto (ponto atual + vel.lins*tempo)
        th(n+1) = th(n)+V(3)*Dt;        %proximo theta (angulo atual + vel.ang*tempo)
    end

    axis equal; hold on; grid on;
    plot(P(1,:), P(2,:), '.');
    set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
    title('Inverse Kinematics', 'FontName', fontName, 'FontSize', fontSize, 'Color', fontColor); 

    
  

end

if ismember('Ex2 ', List)
    %% Ex2 - Test invkinDDxy() e calcular resulting path com varios pontos
    close all;

    %P0 -> 3s -> P1 -> 4s -> P2 -> 5s -> P3 -> 4s -> P0

    %Orientation
    theta0 = 0;

    %Variables
    r=0.1;  %wheel radius
    L=0.5;  %wheel separation
    Dt=0.1; %intervalo tempo
    type=1; %DD

    %Initialize positions and orientationss:
    P=[];       
    theta=[];   
    
    %Declare path points    
    P0 = [0;0];
    
    P1.xy=[3;2];                %Ponto chegada com campos x,y,t
    P1.t=3;

    P2=[5;1];
    P3=[2,-2];

    P=[P P0];                   %Adicionar ponto partida à lista
    theta=[theta theta0];       %Adicionar orientação inicial à lista
 
    % -------------------- Orientation --------------------

    %rotation time
    trot=1; 
    [VR,VL]=invkinDDxy(P0(1), P0(2), L, trot, P1.xy(1), P1.xy(2));  %Roda na orientação do ponto chegada
    
    % number of iter
    ST=trot/Dt;
    
    for n=1:ST   
        [Vx,Vy,w]=localvel(type,r,L,VR/r,VL/r,0);   %velocidade angular das rodas
        R=orm(theta(n));
        V=inv(R)*[Vx;Vy;w];
        %P(:,n+1)=P(:,n)+V(1:2)*Dt;
        P=[P P(:,n)+V(1:2)*Dt];         %criar trajetoria
        %theta(n+1)=theta(n)+V(3)*Dt;
        theta=[theta theta(n)+V(3)*Dt]; %criar trajetoria
    end
    
    %-------------------- Linear Traj --------------------
    
    [VR,VL]=invkinDDxy(P1.xy(1), P1.xy(2), L, P1.t, 0, 0);
    
    % number of iter
    ST=P1.t/Dt;
    
    for n=1:ST   
        [Vx,Vy,w]=localvel(type,r,L,VR/r,VL/r,0);  %velocidade angular rodas
        R=orm(theta(end));
        V=inv(R)*[Vx;Vy;w];
        %P(:,n+1)=P(:,n)+V(1:2)*Dt;
        P=[P P(:,end)+V(1:2)*Dt];           
        %theta(n+1)=theta(n)+V(3)*Dt;
        theta=[theta theta(end)+V(3)*Dt];     %end - utilizar o ultimo ponto
    end
    
    
    axis equal; grid on; hold on;
    plot(P(1,:),P(2,:),'*');   %X - Todas colunas linha 1; Y - Todas colunas linha 2;
    set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
    title('Inverse Kinematics Path', 'FontName', fontName, 'FontSize', fontSize, 'Color', fontColor); 




    
end


