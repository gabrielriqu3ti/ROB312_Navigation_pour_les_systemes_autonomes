clear % effacer toutes les variables
close all % fermer toutes les fenêtres
clc % effacer la ligne de commande
rng(123456) % imposer la graine de génération de nombres pseudo-aléatoire pour la répétabilité

% Paramètres initiaux et de simulation
P_hat = diag([10, 10, 10, 3, 3, 1].^2); % matrice de covariance initiale
X_hat = [0, 0, 100, 5, 5, 0]'; % estimé initial
d = size(X_hat,1); % dimension de l'état
X_reel = X_hat + sqrtm(P_hat)*randn(d,1); % état vrai (inconnu du filtre)
dt = 0.1; % pas de temps
F = eye(d); % matrice de dynamique (à compléter)
F(1,4) = dt;
F(2,5) = dt;
F(3,6) = dt;
Q = diag([0.2,0.2,0.2,0.01,0.01,0.001]); % matrice de covariance de bruit de dynamique
H = zeros(3,d); % matrice d'observation (à compléter)
H(1,1) = 1;
H(2,2) = 1;
H(3,3) = 1;
R = diag([10,10,10]); % matrice de covariance du bruit de mesure
dm = size(R,1); % dimension du vecteur de mesures
K = zeros(d,dm);

% Initialisation des variables de stockage des données
tk=1;
t_sim(tk) = 0;
K_sim(:,:,tk) = zeros(d,dm);
inno_sim(:,tk) = zeros(dm,1);
P_sim(:,:,tk) = P_hat;
Pdiag_sim(:,:,tk) = diag(P_hat);
Xv_sim(:,tk) = X_reel;
X_hat_sim(:,tk) = X_hat;

% Boucle de simulation
T = 10; % durée (s)
for tk = 2:(T/dt)
    % simulation de l'état vrai
    t = dt*(tk-1); % temps courant
    X_reel = F*X_reel + sqrt(Q)*randn(d,1); % propagation de l'état réel (à compléter)
    
    % prediction (à compléter: variables X_hat et P_hat)
    X_hat = F*X_hat;
    P_hat = F*P_hat*F' + Q;

    % génération de la mesure réelle bruitée (à compléter)
    Y = H*X_reel + sqrt(R)*randn(dm,1);
    
    % validité de la mesure réelle
    if t > 3 && t < 7
        is_measurementValid = false;
    else
        is_measurementValid = true;
    end
    
    % correction (à compléter: variables K, P_hat, inno, X_hat)
    if is_measurementValid
        K = P_hat*H'/(R + H*P_hat*H');
        inno = Y - H*X_hat;
        X_hat = X_hat + K*inno;
        P_hat = (eye(d) - K*H)*P_hat;
    end
    
    % enregistrement des variables (pour plot)
    K_sim(:,:,tk) = K;
    t_sim(tk) = t;
    inno_sim(:,tk) = inno;
    P_sim(:,:,tk) = P_hat;
    Pdiag_sim(:,:,tk) = diag(P_hat);
    Xv_sim(:,tk) = X_reel;
    X_hat_sim(:,tk) = X_hat;
    
    % plot instantané
    figure(2)
    clf
    hold on
    plot(Xv_sim(1,:), Xv_sim(2,:), '-b')
    plot(X_hat_sim(1,:), X_hat_sim(2,:), '-r')
    scatter(Xv_sim(1,tk), Xv_sim(2,tk), 'b')
    scatter(X_hat_sim(1,tk), X_hat_sim(2,tk), '.r')
    plotcov(X_hat_sim(:,tk),3^2*P_sim(:,:,tk),'r')
    xlabel('x (m)')
    ylabel('y (m)')
    legend({'trajectoire vraie', 'trajectoire estimée', 'position vrai', 'position estimée', 'uncertitude(3\sigma)'},'Location','best')
    grid on
    drawnow
end

figure(1)
labels = {'x (m)','y (m)','z (m)','Vx (m)','Vy (m)','Vz (m)'};
for i = 1:d
    subplot(3,2,i)
    hold on
    fill([t_sim flip(t_sim,2)],[X_hat_sim(i,:) - 3*sqrt(Pdiag_sim(i,:)), X_hat_sim(i,end:-1:1) + 3*sqrt(Pdiag_sim(i,end:-1:1))], [7 7 7]/8);
    plot(t_sim, Xv_sim(i,:), 'b')
    plot(t_sim, X_hat_sim(i,:), 'r')
    grid on
    xlabel('time (s)')
    ylabel(labels(i))
end
legend('uncertainty (3\sigma)', 'actual state', 'estimated state')
