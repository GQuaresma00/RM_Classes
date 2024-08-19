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
%         'Ex3 '
%         'Ex4'
        };
%==========================================================================        

if ismember('Ex1 ', List)
    %% Ex1 - Localization from motion-based triangulation
    close all;
    
    %1.a)
    %robot initial position (50º)
    P1 = [2;0]; %vetor - coluna
    %beacon coordinates
    B = [0;5];  % vetor - coluna
    %theta 50º - radians
    theta = 50/180*pi;
    %get alpha from B,P
    alpha = getbdir(B,P1,theta);
    
    %1.b e 1.c)
    V = 2;
    dt = 0.2;
    w = 0;
    T = 5;

    axis equal; hold on
    grid on
    plot(B(1),B(2),'.r','MarkerSize',30)
    line([0 0], [0,8]);
    line([0 10], [0 0]);
 
    %num total passos
    NN = T/dt; 
    %vetor para as posicoes
    P = P1;

    for n=2:NN
        % Odometry estimate of P position
        P(:,n) = P(:,n-1)+V*dt*[cos(theta) ; sin(theta)];  %vetor coluna
        bheta = pi-getbdir(B,P(:,n),theta); % o bheta é o ang interno, como medimos o angulo externo: pi-bheta'
        L1 = norm(P(:,n)-P(:,1)); %L1 - distancia entre P1 e o P(n)
        L2 = L1*(sin(alpha)/sin(alpha+bheta));
    
        %Localization results:
        %Estimated (x1,y1) using L2 and bheta        
        x1 = B(1)+L2*cos(theta-bheta);
        y1 = B(2)+L2*sin(theta-bheta);
        plot(x1,y1,'r*')
        %Calcula-se outra vez o ponto, devido ao bheta poder ter erro
        
        %Erro relativo da distancia calculada (1.d)
        Rel_Err(n)=abs(L2-norm(B-P(:,n))/norm(B-P(:,n)));
        
        %Diferenca entre distancia real e calculada
        Err(n) = norm(P(:,n)-[x1;y1]);
        
        h = line([B(1) x1], [B(2) y1]);
        h.Color = 'g';

    end

    plot(P(1,:),P(2,:),'ob'); %todos os x e todos os y
    figure(2)
    plot(Err)
    figure(3)
    plot(Rel_Err)

      
end

if ismember('Ex2 ', List)
    %% Ex2 - Localization with 2 beacons
    close all;
    
    syms th1 th2 x y x1 y1 x2 y2 real
    
    % 
    M = [-sin(th1) cos(th1)
         -sin(th2) cos(th2)];

    % valores conhecidos
    V = [-x1*sin(th1)+y1*cos(th1)
         -x2*sin(th2)+y2*cos(th2)];

    P = simplify(inv(M)*V) %solution [x;y]
    pretty(P)

    %ex2.b
    
    %beacons coordinates
    x1 = 10;
    y1 = 6;
    x2 = 5;
    y2 = 5;
    B1 = [x1;y1];
    B2 = [x2;y2];
    %estimated point
    PE = [8;2];
    % simulate measurement
    V1 = B1-PE;
    V2 = B2-PE;
    th1 = atan2(V1(2),V1(1));
    th2 = atan2(V2(2),V2(1));
    
    %substitute every symbol to last numeric
    Q=subs(P);

    %variation of arithmetic precision
    vpa(Q,3)
    %output: 8.0 2.0 same as PE



    %% EX 3 - GDOP 

%     x = P(1);
%     y = P(2);
%     % voltar a converter em simbolico th1 e th2
%     syms th1 th2
%     J = simplify(jacobian([x y], [th1 th2]));
% 
%     B1 = [10;6];
%     B2 = [5;5];
%     
%     x1 = B1(1); y1 = B1(2);
%     x2 = B2(1); y2 = B2(2);
%     
%     th1 = subs(th1);
%     th2 = subs(th2);
% 
%     J1 = subs(J);
%     GDOP = vpa(abs(simplify(det(J1))),5)
%     fsurf(GDOP, [-pi pi -pi pi])
%     zlim([0 100])


end


if ismember('Ex3 ', List)
    %% Ex3 - Uncertainty and GDOP
    close all;
    
end
