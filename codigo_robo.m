%% UNIVERSIDADE FEDERAL DE SANTA CATARINA %%

% ALUNOS: ANA JÚLIA LANZARIN E EDUARD HERMES ANSCHAU
% PROFESSOR: MARCELO ROBERTO PETRY
% INTRODUÇÃO À ROBÓTICA INDUSTRIAL - TRABALHO 1
% SIMULAÇÃO DO ROBÔ KUKA KR3 R540

clc, clear all, close all

%% Atribuição dos parâmetros DH ao robô através da função Link %%

L(1) = Link('d', -345, 'a', 20, 'alpha', pi/2, 'qlim', [-170*pi/180 170*pi/180]);
L(2) = Link('offset', -pi/2, 'd', 0, 'a', 260, 'alpha', 0, 'qlim', [-170*pi/180 50*pi/180]);
L(3) = Link('d', 0, 'a', 20, 'alpha', pi/2, 'qlim', [-155*pi/180 110*pi/180]);
L(4) = Link('d', -260, 'a', 0, 'alpha', -pi/2, 'qlim', [-175*pi/180 175*pi/180]);
L(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-120*pi/180 120*pi/180]);
L(6) = Link('d', -195, 'a', 0, 'alpha', 0, 'qlim', [-350*pi/180 350*pi/180]);

KUKA = SerialLink(L, 'name', 'KUKA KR3 R540'); %%cria um robô de links em série com os links criados anteriormente

%% OBTENÇÃO DA CINEMÁTICA DIRETA ATRAVÉS DAS FUNÇÕES DA TOOLBOX DE ROBÓTICA %% 

%O usuário fornece os valores desejados de q1, q2, q3, q4, q5 e q6:
q1 = deg2rad(input ('Forneca o angulo da junta 1 ([-170,170] graus): ')); 
q2 = deg2rad(input ('Forneca o angulo da junta 2 ([0, 50] graus): ')); 
q3 = deg2rad(input ('Forneca o angulo da junta 3 ([0, 94] graus): ')); 
q4 = deg2rad(input ('Forneca o angulo da junta 4 ((0, 175] graus): ')); 
q5 = deg2rad(input ('Forneca o angulo da junta 5 ((0, 120] graus): ')); 
q6 = deg2rad(input ('Forneca o angulo da junta 6  ((0, 180] graus): ')); 

%%--> Criando as matrizes de transformação homogêneas com as funções do toolbox:
T1 = L(1).A(q1).T;
T2 = L(2).A(q2).T;
T3 = L(3).A(q3).T;
T4 = L(4).A(q4).T;
T4(1,2) = 0;
T4(2,2) = 0;        
T4(3,3) = 0;
T5 = L(5).A(q5).T;
T5(1,2) = 0;
T5 (2,2) = 0;
T5(3,3) = 0;
T6 = L(6).A(q6).T;

Q = [q1 q2 q3 q4 q5 q6];

set(gca,'Zdir','reverse','Xdir','reverse') %Aqui se faz a reversão dos eixos Z e X
                                           %Para o robô manter uma postura
                                           %''em pé''
KUKA.teach(Q, 'notiles', 'floorlevel', 150, 'lightpos',[0 0 -20])
%A função teach cria a simulação gráfica do robô. É possível observar que
%alteramos o nível do chão na simulação, para melhor visualização

M = T1*T2*T3*T4*T5*T6; %%Faz a cinemática direta

clc
%%Mostra na tela as coordenadas X,Y e Z ao usuário:
disp('As coordenadas X,Y e Z do efetuador sao: ');
disp(M(1:3,4));


%% A CINEMÁTICA INVERSA %%

 M = T1*T2*T3*T4*T5*T6; %%Esta matriz será conhecida
 
 d6=[0; 0; 195]; %%d6 é um comprimento no eixo Z (por convenção) de 195mm, entre o referencial
 %do punho e o referencial da ferramenta
 
 p06 = M(1:3,4); %%Vetor que une o referencial inercial ao referencial da ferramenta
 R = M(1:3,1:3); %%Submatriz de rotação de M
 p0w_aux = p06+R*d6; %%Vetor que une o referencial inercial ao referencial do
                     %%punho
 q1 = atan2(p0w_aux(2,1),p0w_aux(1,1)); %%Cálculo de q1
 
 %%As variáveis definidas com aux são os vetores propriamente ditos,
 %%As variáveis definidas são os comprimentos destes vetores.
 %%Todas essa variáveis são calculadas através do método geométrico
 %%esboçado no relatório do trabalho.
 p0w = sqrt(p0w_aux(1,1)^2+p0w_aux(2,1)^2+p0w_aux(3,1)^2);
 p01_aux = L(1).A(q1);
 p01_aux = [p01_aux.t(1,1); p01_aux.t(2,1); p01_aux.t(3,1)];
 p01 = sqrt(p01_aux(1,1)^2+p01_aux(2,1)^2+p01_aux(3,1)^2); 
 p1w_aux = p0w_aux-p01_aux;
 p1w = sqrt(p1w_aux(1,1)^2+p1w_aux(2,1)^2+p1w_aux(3,1)^2);
 p23 = sqrt(260^2+20^2);
 
 %%Os ângulos auxiliares necessários para o cálculo de q2:
 beta = atan(345/20);
 psi = acos((p0w^2-p01^2-p1w^2)/(-2*p01*p1w));
 omega = acos((-p1w^2-260^2+p23^2)/(-2*260*p1w));
 
 if rad2deg(psi)>180+atand(20/345)+0.000000000000001e+02
    disp('Ponto invalido');
 end
 
 q2 = 2*pi-(pi/2+beta+psi+omega);
 
 %%Os ângulos auxiliares necessários para o cálculo de q3:
 epsilon = acos((p1w^2-260^2-p23^2)/(-2*260*p23));
 phi = atan(260/20);
 
 if rad2deg(epsilon)>180 || rad2deg(epsilon)<0
    disp('Ponto invalido');
 end
 
 q3 = pi-epsilon-phi;
 
 %%A orientação, pelo desacoplamento cinemático:
 T1 = L(1).A(q1).T;
 T2 = L(2).A(q2).T;
 T3 = L(3).A(q3).T;
 T03 = (T1*T2*T3);
 T36 = inv(T03)*M;
 
 %%Cálculos de q4, q5 e q6:
 q4 = atan2(T36(2,3),T36(1,3));
 q5 = atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
 q6 = atan2(T36(3,2),-T36(3,1));
 
 %%Só converte os ângulos achados de radianos para graus
 Q = [rad2deg(q1); rad2deg(q2); rad2deg(q3); rad2deg(q4); rad2deg(q5); rad2deg(q6)];
 
 %%TRATAMENTO DE ERRO PARA CINEMÁTICA INVERSA (feito através de experimentos com o algoritmo):
 error = 0;
if q1 < -170*pi/180 || q1> 170*pi/180
    disp('Nao eh possivel alcancar este ponto')
    error = 1;
end

if q2 < -170*pi/180 || q2> 50*pi/180
    disp('Nao eh possivel alcancar este ponto')
    error = 1;
end

if q3 < -110*pi/180 || q3> 155*pi/180
    disp('Nao eh possivel alcancar este ponto')
    error = 1;
end

if q4 < -175*pi/180 || q4> 175*pi/180
    disp('Nao eh possivel alcancar este ponto')
    error = 1;
end

if q5 < -120*pi/180 || q5> 120*pi/180
    disp('Nao eh possivel alcancar este ponto')
    error = 1;
end

if q6 < -350*pi/180 || q6> 350*pi/180
    disp('Nao eh possivel alcancar este ponto')
    error = 1;
end

if error ~= 1
    disp('Os angulos das juntas resultam em:')
    disp(Q)
end