%% Clear and format
                                                                            % Close files and Clear Variables.            
clc; close all; clearvars; workspace;                                       % Make sure the workspace panel is showing.
fontSize=40; fontName='Impact'; fontColor=[0.15 0.22 0.33];                 % Format font size, style and color.     
format long g; format compact;                                              % Long Numeric Values. Supress Blank Lines(Display).
addpath ../lib;

%% Aula2

close
clear

%% Velocidades constantes:

% %Inicializar parametros constantes:
% ST=2000;
% w1=1.01;               %1.5 - curva  %1.01 - arco circunferência para config1.
% aw2=1;
% w3=0;
% r=1;
% L=4;
% 
% %Inicializar lista de posições e orientações:
% P=zeros(2,ST+1);        %2 linhas com ST colunas + coluna inicial
% theta=zeros(1,ST+1);    %2 linhas com ST colunas + coluna inicial
% 
% P0=[0;0];               %Ponto partida
% theta0=0;               %Orientação inicial
% 
% P(:,1)=P0;              %Adicionar ponto partida à lista
% theta(1)=theta0;        %Adicionar orientação inicial à lista
% Dt=1;                   %Intervalo de tempo
% 
% 
% t=1;                    %1 config. - w1=1 reta           %DD
% t=2; aw2=-pi/4;         %2 config. - vira para direita   %TRI
% t=3; w=1; aw2=1; w3=1;  %3 config. - 1 ponto(ver ori)    %OMNI
% 
% for n=1:ST      
%     [Vx,Vy,w]=localvel(t,r,L,w1,aw2,w3);
%     R=orm(theta(n));
%     V=inv(R)*[Vx;Vy;w];
%     P(:,n+1)=P(:,n)+V(1:2)*Dt;
%     theta(n+1)=theta(n)+V(3)*Dt;
% end
% 
% axis equal; grid on; hold on;
% plot(P(1,:),P(2,:),'*');   %X - Todas colunas linha 1; Y - Todas colunas linha 2;
% % plot(mod(theta,2*pi),'.')  %Visualizar orientação OMNI config 3


%% Velocidades variáveis:

%Inicializar parametros constantes:
ST=500;             %Visualizar + resultados(500) / 1000,2000 ....
w1=1;               %1.5 - curva  %1.01 - arco circunferência para config1.
aw2=1;
w3=0;
r=1;
L=4;

%Inicializar lista de posições e orientações:
P=zeros(2,ST+1);        %2 linhas com ST colunas + coluna inicial
theta=zeros(1,ST+1);    %2 linhas com ST colunas + coluna inicial

P0=[0;0];               %Ponto partida
theta0=0;               %Orientação inicial

P(:,1)=P0;              %Adicionar ponto partida à lista
theta(1)=theta0;        %Adicionar orientação inicial à lista
Dt=0.1;                 %Intervalo de tempo
%Começa a voltar para trás - 0.5

t=1;                      %1 config.    %DD
% t=2; aw2=-pi/4;         %2 config.    %TRI
% t=3; w=1; aw2=1; w3=1;  %3 config.    %OMNI

for n=1:ST
    
    %Velocidades variaveis
    w1=3*sin(n*2*pi/ST);
%     aw2=2*sin(n*2*pi/ST);
    aw2=2*sin(3*n*2*pi/ST); %Frequencias diferentes
    w3=0;
    
    [Vx,Vy,w]=localvel(t,r,L,w1,aw2,w3);
    R=orm(theta(n));
    V=inv(R)*[Vx;Vy;w];
    P(:,n+1)=P(:,n)+V(1:2)*Dt;
    theta(n+1)=theta(n)+V(3)*Dt;
end

axis equal; grid on; hold on;
plot(P(1,:),P(2,:),'*');   %X - Todas colunas linha 1; Y - Todas colunas linha 2;
% plot(mod(theta,2*pi),'.')  %Visualizar orientação OMNI config 3





