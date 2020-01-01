clear % effacer toutes les variables
close all % fermer toutes les fenêtres
clc % effacer la ligne de commande
rng(123457) % imposer la graine de génération de nombres pseudo-aléatoire pour la répétabilité

% Paramètres du filtre
N = 3000; % nombre de particules
P_hat = diag([5000, 5000, 100, 20*pi/180].^2); % matrice de covariance initiale
X_hat = [230000, 90000, 1000, 150*pi/180]'; % estimé initial
d = size(X_hat,1); % dimension de l'état
Qf = diag([3, 3, 0.6, 0.001*180/pi].^2); % matrice de covariance de bruit de dynamique
Rf = 20.^2; % matrice de covariance du bruit de mesure du filtre
threshold_resampling = 0.5; % seuil de ré-échantillonnage (theta_eff)

% Paramètres initiaux et de simulation
erreurInitiale = sqrt(P_hat)*randn(d,1);
X_reel = X_hat + erreurInitiale; % état vrai initial (inconnu du filtre)    
dt = 1; % pas de temps
R = 20.^2; % matrice de covariance du bruit de mesure réelle (inconnu du filtre)
dm = size(R,1); % dimension du vecteur de mesures
is_temporalPlot = true;

% Chargement des données
load carte.mat
params.pasx_reel = pasx_reel;
params.pasy_reel = pasy_reel;
params.nrow_h = size(h_MNT,1);
params.dxh_MNT = diff(h_MNT,1,2)/pasx_reel;
params.dyh_MNT = diff(h_MNT)/pasy_reel;
params.h_MNT = h_MNT;
params.x_MNT = x_MNT;
params.y_MNT = y_MNT;

% Initialisation du filtre
Xp = X_hat*ones(1,N) + sqrt(P_hat)*randn(d,N); % Tirage des particules autours de X_hat initial
wp = 1/N*ones(1,N); % poids initiaux

% Initialisation des variables de stockage des données
tk=1;
t_sim(tk) = 0;
P_sim(:,:,tk) = P_hat;
Pdiag_sim(:,:,tk) = diag(P_hat);
X_reel_sim(:,tk) = X_reel;
X_hat_sim(:,tk) = X_hat;

% Boucle de simulation
T = 300; % durée (s)
for tk = 2:(T/dt)
    % commande (définit la trajectoire)
    V = 300; % Vitesse
    omega = -0.01; % vitesse angulaire (rad/s)
    
    % simulation de l'état vrai (attention, inconnu du filtre)
    t = dt*(tk-1); % temps courant
    X_reel(1) = X_reel(1) + V*dt*cos(X_reel(4)); % propagation de l'état réel (à compléter)
    X_reel(2) = X_reel(2) + V*dt*sin(X_reel(4));
    X_reel(4) = X_reel(4) + omega*dt;
%     X_reel = X_reel + sqrtm(Qf)*randn(d,1);
    X_reel(4) = AngleWrap(mod(X_reel(4), 2*pi)); % modulo 2pi sur le cap
    
    % prediction (à compléter: variables Xp, X_hat et P_hat)
    Xp(1,:) = Xp(1,:) + V*dt*cos(Xp(4,:)); % particules prédites
    Xp(2,:) = Xp(2,:) + V*dt*sin(Xp(4,:));
    Xp(4,:) = Xp(4,:) + omega*dt;
    Xp = Xp + sqrtm(Qf)*randn(d,N);
    X_hat = Xp*wp'; % état estimé prédit
    X_hat(4,:) = AngleWrap(mod(X_hat(4,:), 2*pi)); % modulo 2pi sur le cap
    P_hat = ((Xp-X_hat(:,1*ones(1,N))).*wp(ones(d,1),:) )*(Xp-X_hat(:,ones(1,N)))';
    
    % génération de la mesure réelle (à compléter)    
    Y = X_reel(3) - hobs(X_reel, params) + sqrt(R)*randn(1,1);
    
    % validité de la mesure réelle
    is_measurementValid = true;
    
    % correction (à compléter)
    if is_measurementValid
        for i=1:N
            Yp =  Xp(3,i) - hobs(Xp(:,i),params);
            inno = Y - Yp;
            wp(i) = wp(i)*exp(-inno.^2/(2*Rf));
        end
        n_wp = sum(wp);
        wp = wp/n_wp;
    end
    
    % Ré-échantillonnage
    criterionResampling = 0;
    for wi=1:d
        criterionResampling = criterionResampling + wp(wi)^2;
    end
    criterionResampling = criterionResampling/d; % critère de rééchantillonnage "N efficace" à compléter
    if criterionResampling < N*threshold_resampling
        Xp = Xp(:,select(wp)); % sélection des nouvelles particules selon l'algorithme de ré-échantillonnage multinomial
        wp = 1/N*ones(1,N); % ré-initialisation des poids
    end
    
    % enregistrement des variables (pour plot)
    t_sim(tk) = t;
    P_sim(:,:,tk) = P_hat;
    Pdiag_sim(:,:,tk) = diag(P_hat);
    X_reel_sim(:,tk) = X_reel;
    X_hat_sim(:,tk) = X_hat;
    
    % plot instantané
    if is_temporalPlot
        figure(2)
        clf
        hold on
        imagesc(params.x_MNT*params.pasx_reel/1000, params.y_MNT*params.pasx_reel/1000, h_MNT)
        xlabel('km'); ylabel('km'); 
        title(['Erreur position: ', num2str(norm(X_hat(1:3) - X_reel(1:3))), ' m'])
        grid
        colorbar
        hold on
        plot(X_reel_sim(1,:)./1000,X_reel_sim(2,:)./1000,'.k')
        plot(X_hat_sim(1,:)./1000,X_hat_sim(2,:)/1000,'.r')
        scatter(Xp(1,:)./1000, Xp(2,:)./1000, '.y')
        legend('position vraie', 'position estimée', 'particules')
        scatter(X_reel_sim(1,tk)./1000, X_reel_sim(2,tk)./1000, '.k')
        scatter(X_hat_sim(1,tk)./1000, X_hat_sim(2,tk)./1000, '.r')
        grid on
        axis equal
        if mod(tk,5)
            drawnow
        end
    end
end

% Plot des résultats
figure(1)
labels = {'x (m)','y (m)','z (m)','\theta (rad)'};
for i = 1:d
    subplot(4,1,i)
    hold on
    fill([t_sim flip(t_sim,2)],[X_hat_sim(i,:) - 3*sqrt(Pdiag_sim(i,:)), X_hat_sim(i,end:-1:1) + 3*sqrt(Pdiag_sim(i,end:-1:1))], [7 7 7]/8);
    plot(t_sim, X_reel_sim(i,:), 'b')
    plot(t_sim, X_hat_sim(i,:), 'r')
    grid on
    xlabel('time (s)')
    ylabel(labels(i))
end
legend('uncertainty (3\sigma)', 'actual state', 'estimated state')


