clear % effacer toutes les variables
close all % fermer toutes les fenêtres
clc % effacer la ligne de commande
rng(123456) % imposer la graine de génération de nombres pseudo-aléatoire pour la répétabilité

% Paramètres du filtre
P_hat = diag([10, 10, 10*pi/180].^2); % matrice de covariance initiale
X_hat = [0, 0, 0]'; % estimé initial
d = size(X_hat,1); % dimension de l'état
Qf = diag([0.1,0.1,0.001]); % matrice de covariance de bruit de dynamique
Rfi = diag([10,1*pi/180].^2); % matrice de covariance du bruit de mesure pour chaque amer

% Paramètres initiaux et de simulation
X_reel = X_hat + sqrtm(P_hat)*randn(d,1); % état vrai (inconnu du filtre)    
dt = 0.1; % pas de temps
V = 10;
%Q = diag([0.0001,0.0001,0.01]); % matrice de covariance de bruit de dynamique
%Ri = diag([10,1*pi/180].^2); % matrice de covariance du bruit de mesure pour chaque amer
amers(1).P = [10;00]; % position de l'amer 1 (point de repère)
amers(2).P = [0;50]; % position de l'amer 2 (point de repère)
for i=3:9
    amers(i).P = [9*i*cos(2*pi*i/10);9*i*sin(2*pi*i/10)]; % position de l'amer 1 (point de repère)
end
dm = size(amers,2)*size(Rfi,1); % dimension du vecteur de mesures
H = zeros(dm,d); % matrice d'observation (à compléter)
%R = [];
Rf = [];
for i = 1:size(amers,2)
%    R = blkdiag(R,Ri);
    Rf = blkdiag(Rf,Rfi);
end

% Initialisation des variables de stockage des données
tk=1;
t_sim(tk) = 0;
K = zeros(d,dm);
K_sim(:,:,tk) = K;
inno = zeros(dm,1);
inno_sim(:,tk) = inno;
P_sim(:,:,tk) = P_hat;
Pdiag_sim(:,:,tk) = diag(P_hat);
X_reel_sim(:,tk) = X_reel;
X_hat_sim(:,tk) = X_hat;

% Boucle de simulation
T = 20; % durée (s)
for tk = 2:(T/dt)
    % commande (définit la trajectoire)
    V = 10; % Vitesse
    omega = cos(tk*2*pi/100); % vitesse angulaire
    
    % simulation de l'état vrai (attention, inconnu du filtre)
    t = dt*(tk-1); % temps courant
    X_reel(1,1) = X_reel(1,1) + V*dt*cos(X_reel(3,1)); % propagation de l'état réel (à compléter)
    X_reel(2,1) = X_reel(2,1) + V*dt*sin(X_reel(3,1));
    X_reel(3,1) = X_reel(3,1) + omega*dt;
    X_reel = X_reel + sqrtm(Qf)*randn(d,1);
    X_reel(3,1) = AngleWrap(mod(X_reel(3,1), 2*pi));
    
    % prediction (à compléter: variables X_hat et P_hat)
    X_hat(1,1) = X_hat(1,1) + V*dt*cos(X_hat(3,1)); % état estimé prédit
    X_hat(2,1) = X_hat(2,1) + V*dt*sin(X_hat(3,1));
    X_hat(3,1) = X_hat(3,1) + omega*dt;
    X_hat(3,1) = AngleWrap(mod(X_hat(3,1), 2*pi)); % modulo sur le cap
    F = eye(d); % matrice de dynamique
    F(1,3) = - V*dt*sin(X_hat(3,1));
    F(2,3) = + V*dt*cos(X_hat(3,1));
    P_hat = F*P_hat*F' + Qf;
    
    % génération de la mesure réelle (à compléter)    
    Y = zeros(dm,1);
    for i = 1:size(amers,2)
        P_amer = amers(i).P;
        Delta = P_amer(1:2,1)-X_reel(1:2,1); % attention, Delta réel inconnu du filtre
        Y(2*i-1,1) = sqrt(Delta(1,1)^2 + Delta(2,1)^2); % mesure sur l'amer i, à compléter
        Y(2*i,1) = atan2(Delta(2,1),Delta(1,1)) - X_reel(3,1); % mesure sur l'amer i, à compléter
        Y(2*i-1:2*i,1) = Y(2*i-1:2*i,1);
        Y(2*i,1) = mod(Y(2*i,1),2*pi); % modulo sur le cap relatif
    end
    Y = Y + sqrtm(Rf)*randn(dm,1);
    for i=1:dm/2
        Y(2*i,1) = mod(Y(2*i,1),2*pi);
    end
    
    % validité de la mesure réelle
%      is_measurementValid = false;
     is_measurementValid = true;
%      if (mod(t,1) == 0)
%         is_measurementValid = true;
%      end
    
    % correction (à compléter: variables K, P_hat, inno, X_hat)
    if is_measurementValid
        H = zeros(dm,d);
        Y_hat = zeros(dm,1);
        for i = 1:size(amers,2)
            P_amer = amers(i).P;
            Delta_hat = P_amer(1:2,1) - X_hat(1:2,1);
            H(2*i-1,1) = - Delta_hat(1,1)/sqrt(Delta_hat(1,1)^2 + Delta_hat(2,1)^2);
            H(2*i-1,2) = - Delta_hat(2,1)/sqrt(Delta_hat(1,1)^2 + Delta_hat(2,1)^2);
            H(2*i,1) = + Delta_hat(2,1)/(Delta_hat(1,1)^2 + Delta_hat(2,1)^2);
            H(2*i,2) = - Delta_hat(1,1)/(Delta_hat(1,1)^2 + Delta_hat(2,1)^2);
            H(2*i,3) = -1;
            Y_hat(2*i-1,1) = sqrt(Delta_hat(1,1)^2 + Delta_hat(2,1)^2);
            Y_hat(2*i,1) = atan2(Delta_hat(2,1), Delta_hat(1,1)) - X_hat(3,1);
            Y_hat(2*i,1) = mod(Y_hat(2*i,1), 2*pi);
            inno(2*i-1:2*i,1) = Y(2*i-1:2*i,1) - Y_hat(2*i-1:2*i,1);
            inno(2*i,1) = AngleWrap(mod(inno(2*i,1), 2*pi));
        end
        K = P_hat*H'/(Rf + H*P_hat*H');
        X_hat = X_hat + K*inno;
        X_hat(3,1) = AngleWrap(mod(X_hat(3,1), 2*pi));
        P_hat = (eye(d) - K*H)*P_hat;
    end
    
    % enregistrement des variables (pour plot)
    K_sim(:,:,tk) = K;
    t_sim(tk) = t;
    inno_sim(:,tk) = inno;
    P_sim(:,:,tk) = P_hat;
    Pdiag_sim(:,:,tk) = diag(P_hat);
    X_reel_sim(:,tk) = X_reel;
    X_hat_sim(:,tk) = X_hat;
    
    % plot instantané
    figure(2)
    clf
    hold on
    plot(X_reel_sim(1,:), X_reel_sim(2,:), '-b')
    plot(X_hat_sim(1,:), X_hat_sim(2,:), '-r')
    scatter(X_reel_sim(1,tk), X_reel_sim(2,tk), 'b')
    scatter(X_hat_sim(1,tk), X_hat_sim(2,tk), '.r')
    plotcov(X_hat_sim(:,tk),3^2*P_sim(:,:,tk),'r')
    lgd = {'trajectoire vraie', 'trajectoire estimée', 'position vrai', 'position estimée', 'uncertitude(3\sigma)'};
    for i = 1:size(amers,2)
        P_amer = amers(i).P;
        scatter(P_amer(1), P_amer(2), 'ok')
        Delta = P_amer(1:2)-X_reel(1:2);
        plot([X_reel_sim(1,tk), X_reel_sim(1,tk) + Delta(1)], [X_reel_sim(2,tk), X_reel_sim(2,tk) + Delta(2)], 'g');
        lgd{1,3+2*i} = "position amer " + i;
        lgd{1,4+2*i} = "distance amer " + i;
    end
    legend(lgd,'Location','best')
    xlabel('x (m)')
    ylabel('y (m)')
    grid on
    axis equal
    drawnow
end

figure(1)
labels = {'x (m)','y (m)','\theta (rad)'};
for i = 1:d
    subplot(3,1,i)
    hold on
    fill([t_sim flip(t_sim,2)],[X_hat_sim(i,:) - 3*sqrt(Pdiag_sim(i,:)), X_hat_sim(i,end:-1:1) + 3*sqrt(Pdiag_sim(i,end:-1:1))], [7 7 7]/8);
    plot(t_sim, X_reel_sim(i,:), 'b')
    plot(t_sim, X_hat_sim(i,:), 'r')
    grid on
    xlabel('time (s)')
    ylabel(labels(i))
end
legend('uncertainty (3\sigma)', 'actual state', 'estimated state')


