function conf = IK_RX90(p, R, q_k)
% Petite fonction pour l'IK du robot RX90
% p = position de l'effecteur final
% R = orientation (matrice 3x3)
% q_k = configuration courante qu'on veut retrouver si possible

    if nargin < 3 || isempty(q_k)
        q_k = zeros(6,1);
    end
    q_k = q_k(:)';

    [L2, L3, L6, dh] = RX90data();

    p = p(:);

    w = p - L6 * R(:,3);   % centre du poignet
    rho = hypot(w(1), w(2));

    z = w(3);

    q_list = [];

    % deux positions possibles autour de l'axe 1
    base_ang = atan2(w(2), w(1));
    q1_all = [base_ang, base_ang + pi];

    % loi des cos pour le triangle formé par L2 et L3
    cos_q3 = (rho^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3);
    if abs(cos_q3) > 1 + 1e-6
        conf = NaN(6,1);
        return;
    end
    cos_q3 = min(max(cos_q3, -1), 1);  % pour éviter les erreurs numériques
    q3_all = [acos(cos_q3), -acos(cos_q3)];

    for iq1 = 1:length(q1_all)
        q1 = wrap_angle(q1_all(iq1));

        for iq3 = 1:length(q3_all)
            q3 = wrap_angle(q3_all(iq3));

            k1 = L2 + L3 * cos(q3);
            k2 = L3 * sin(q3);
            q2 = atan2(z, rho) - atan2(k2, k1);
            q2 = wrap_angle(q2);

            q123 = [q1, q2, q3];

            T03 = modele_geom(dh(1:3,:), q123);
            R03 = T03(1:3,1:3);
            R36 = R03' * R;

            s5 = hypot(R36(1,3), R36(2,3));
            if s5 > 1e-6
                q5 = atan2(s5, R36(3,3));
                q4 = atan2(R36(2,3), R36(1,3));
                q6 = atan2(R36(3,2), -R36(3,1));
            else
                % Cas aligné, on prend quelque chose de simple
                q5 = 0;
                q4 = 0;
                q6 = atan2(-R36(1,2), R36(1,1));
            end

            sol = wrap_angle([q1, q2, q3, q4, q5, q6]);
            q_list = [q_list; sol]; %#ok<AGROW>

            % solution mirroir du poignet (flip q5)
            sol2 = wrap_angle([q1, q2, q3, q4 + pi, -q5, q6 + pi]);
            q_list = [q_list; sol2]; %#ok<AGROW>
        end
    end

    if isempty(q_list)
        conf = NaN(6,1);
        return;
    end

    % petite suppression des doublons en arrondissant, pas super pro mais ça marche
    rounded = round(q_list, 6);
    [~, idx_unique] = unique(rounded, 'rows');
    q_list = q_list(sort(idx_unique), :);

    d = vecnorm(wrap_angle(q_list - q_k), 2, 2);
    [~, best] = min(d);

    conf = q_list(best, :)';
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

