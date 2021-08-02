%%%% Simulação Catamaran %%%%
% PROBLEMA: NO LONGO PRAZO,ACUMULO DE ENERGIA FAZ ONDAS PARAREM

clearvars, close, clc

addpath(genpath(pwd))

%% Representação do Barco

boat = Catamara;

% Setting Position
boat.pPesoVirtual = -0.25;
boat.pPos.X([1 2 3 6]) = [100, 81, 0, 0]; % [25 60 0]; % CADload/100

% Pontos de Contato 
% Pontos pertencentes ao catamara antigo!
% A fazer: 
%   - Implementar metodo automático para selecionar pontos
%   - Detectar pontos frente/tras/esquerda/direita automaticamente

p1 = [-8.7668         0    0.4510]';
p2 = [-8.4516   -6.9723   -0.2276]';
p3 = [-8.4516    6.9723   -0.2276]';
p4 = [-2.0698   -8.8348    0.7670]';
p5 = [-2.0698    8.8348    0.7670]';
p6 = [ 3.1767   -9.4437    0.8877]';
p7 = [ 3.1767    9.4437    0.8877]';
p8 = [ 9.06195        0   -1.1997]';

originalPdC = [p1 p2 p3 p4 p5 p6 p7 p8];

originalfrente = [-8.7668   -8.4516   -8.4516;
                0   -6.9723    6.9723;
           0.4510   -0.2276   -0.2276];
     
originaltras = [ 3.1767    3.1767    9.0619;
        -9.4437    9.4437         0;
         0.8877    0.8877   -1.1997];
    
originaldireita = [-8.4516  -2.0698 3.1767;
            6.9723   8.8348 9.4437;
           -0.2276   0.7670 0.8877];

originalesquerda = [ -8.4516    3.1767   -2.0698;  
            -6.9723    -9.4437   -8.8348;
            -0.2276    0.8877     0.7670];
        
%% Definindo a Figura que irá rodar a simulação

f1 = figure('Name','Simulação: Catamarã','NumberTitle','off');
f1.Position = [9 2 930 682];

figure(f1);

ax = gca;
ax.FontSize = 12;
xlabel({'$$x$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$y$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$z$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
view(3)
view(45,30)
grid on
hold on
grid minor
light;

% axis([-1 1 -1 1 0 1])
axis tight

set(gca,'Box','on');
set(gca, 'Color', 'none')

% First Plot
hold on

boat.mCADplot();
hold off

pause(0.5)

%% Inicializando superficie:

n=200; % limites (X,Y) em cm(?)
H=zeros(n,n)-2; % gera area 

% hold on
% mesh(H);
% hold off
axis([1 n 1 n -15 25]);


% Energia cinetica e potencial
Ekin=[]; Epot=[];

oldH=H; % ?
newH=H; % ?
i = 2:n-1; % Vetor de 2 a 99 com passo 1;
j = 2:n-1; % Vetor de 2 a 99 com passo 1;
hold on
h=surf(newH); % ?
hold off


% Estilizando superficie
lighting phong;
material shiny;
colormap winter;
lightangle(-45,30)


% Inicializando variaveis (add_Catamara)

% globOH = 0; % ?

% Colocar Catamara na superfície:
[H,globOH] = add_Catamara(boat.pPos.X(1),boat.pPos.X(2),n,H,boat.pPesoVirtual);
boat.pPos.X(3) = mean(H((-10 + round(boat.pPos.X(1),0)):(10 + round(boat.pPos.X(1),0)),...
                           (-10 + round(boat.pPos.X(2),0)):(10 + round(boat.pPos.X(2),0))),'all');
[H,globOH] = add_Catamara(0,0,n,H,boat.pPesoVirtual);

t = tic; % temporizador de interação onda-catamara
total = tic;
tmax = 300;
onda = tic;

Ekin=[];
Epot=[];

while toc(total)<tmax        
    %% Atualiza posição/estado da onda e barco:
                          % n,i,j,dt,c,k,H,oldH,fix,cont,connect
    [aceleracoes,newH,Ekin,Epot]=Wave(n,i,j,0.05,12,0.2,H,oldH,0,0,0,Ekin,Epot);
    
    boat.pPos.X(3) = 1.5*mean(newH((-10 + round(boat.pPos.X(1),0)):(10 + round(boat.pPos.X(1),0)),...
                       (-10 + round(boat.pPos.X(2),0)):(10 + round(boat.pPos.X(2),0))),'all');

    %% Atualiza posicao dos pontos de contato
    % - A fazer: implementar cinemática completa para PdC
    % (atualmente não rotacionam)
    
    PdC(1:3,:) = boat.pPos.X(1:3)+originalPdC(1:3,:);
    frente(1:3,:) = boat.pPos.X(1:3)+originalfrente(1:3,:);
    tras(1:3,:) = boat.pPos.X(1:3)+originaltras(1:3,:);
    esquerda(1:3,:) = boat.pPos.X(1:3)+originalesquerda(1:3,:);
    direita(1:3,:) = boat.pPos.X(1:3)+originaldireita(1:3,:);

    % Encontrar os PdC na superfície da agua:
    indices_PdC = round(PdC,0);
    

    
    %% Direcionando o barco 
    % Atualmente utiliza a maior aceleracao
    % - A fazer: implementar dinamica de braço de alavanca nos PdC do barco
    % (Temos a aceleração em Z em cada um dos PdC, dado isso,
    %    qual a  rotação resultante?)
    
    for ponto=1:size(indices_PdC,2)
        noBarco(ponto) = aceleracoes(indices_PdC(1,ponto),indices_PdC(2,ponto));
    end
    
    resultante = find(noBarco == max(noBarco),1,'first');

    
    % Lógica provisória para movimentar o barco:
    ganho = 0.00001;
    ganho2 = 0.01;
    
    if sum(sum(PdC(resultante))) == 0
%         boat.pSC.Ud(1:2) = [0 0]; 
    else
        if find(frente== PdC(resultante))
            %ir para tras
%             boat.pSC.Ud(1) =boat.pSC.Ud(1) +0.5;
            boat.pPos.X(10) = boat.pPos.X(10)+ganho*(boat.pPos.X(10)-boat.pPar.Xr(10))/boat.pPar.Ts +ganho2;
            
        end    
        if find(PdC(resultante)==tras)
            %ir para frente
%             boat.pSC.Ud(1) =boat.pSC.Ud(1) -0.5;
            boat.pPos.X(10) = boat.pPos.X(10)-ganho*(boat.pPos.X(10)-boat.pPar.Xr(10))/boat.pPar.Ts +ganho2;
            
        end    
        if find(PdC(resultante)==esquerda)
            %ir para direta
%             boat.pSC.Ud(2) =boat.pSC.Ud(2) +0.5;
            boat.pPos.X(11) = boat.pPos.X(11)+ganho*(boat.pPos.X(11)-boat.pPar.Xr(11))/boat.pPar.Ts +ganho2;
            
        end    
        if find(PdC(resultante)==direita)
            %ir para esquerda
%             boat.pSC.Ud(2) =boat.pSC.Ud(2) -0.5;
            boat.pPos.X(11) = boat.pPos.X(11)-ganho*(boat.pPos.X(11)-boat.pPar.Xr(11))/boat.pPar.Ts +ganho2;
            
        end
    end
    
%   boat.pSC.Ud(1):
%     pitch          | [-1,1] <==> [-15,15] degrees
%   boat.pSC.Ud(2):
%     roll           | [-1,1] <==> [-15,15] degrees





    %% Interacao onda-barco
    % Altera comportamento a agua a partir dos pontos de contato e da
    % posicao do barco. 
    % ! Da pra melhorar...
    
    if toc(total)>3 % esperar "transitorio"
        tempo = 0.08*abs(cos(toc(total)./20));
        forca = -0.1*mean(newH,'all')*abs(cos(toc(total)./10));
        
%         if toc(total)>10 && rem(toc(total),5) ==0
%             forca = forca*5;
% %             forca*1.2*exp(toc(total)/tmax);
%         end
        
        if toc(onda)>12
            [H,globOH] = add_Catamara(0,0,n,newH,boat.pPesoVirtual);
            disp('onda')
            onda = tic;
        end

        if toc(t)>tempo
           [newH,globOH] = add_Catamara(boat.pPos.X(1),boat.pPos.X(2),n,newH,boat.pPos.X(3)+forca);
           
           %Ta ruim: (feio dinamicamente)
           for ponto = 1:size(indices_PdC,2)
               [newH,globOH] = add_Catamara(indices_PdC(1,ponto),indices_PdC(2,ponto),n,newH,forca*(1/size(indices_PdC,2)));
%              [newH,globOH] = add_Catamara(indices_PdC(1,ponto)-3*sign(indices_PdC(1,ponto)),indices_PdC(2,ponto)-3*sign(indices_PdC(1,ponto)),n,newH,forca*(1/size(indices_PdC,2)));
           end
           
           t= tic; 
        end
    end 
    

    sDynamicModel(boat);
    %% Desenho dos objetos:
    
    %Mostra Pontos de Contato: 
    %(pontos da agua que tocam no barco e vice-versa)
    hold on
    try
        delete(pontosAgua)
        delete(pontosBarco)
    end
    % PdC do barco (manualmente selecionados)
%     pontosBarco = plot3(PdC(1,:),PdC(2,:),PdC(3,:),'ro','LineWidth', 12,'MarkerSize', 10);
    % PdC da agua (calculados)
%     pontosAgua = plot3(indices_PdC(1,:),indices_PdC(2,:),newH(indices_PdC(1,:),indices_PdC(2,:)),'b*','LineWidth', 12,'MarkerSize', 10);
    %Mostra barco e onda:
    boat.mCADplot();
        
    set(h,'zdata',newH);

    oldH=H;
    H=newH;
    
    pause(0.05);
    
end


 
 
 


 
 
 
