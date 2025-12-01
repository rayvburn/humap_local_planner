% computeThetaAlphaBetaAngle2014

n_alpha1 = [1.0 0 0];
d_alpha_beta1 = [2.0 3.0 1.0];
phi = compute(n_alpha1, d_alpha_beta1)

function phi_alpha_beta = compute(n_alpha, d_alpha_beta)
    angle_n_alpha = atan2(n_alpha(2), n_alpha(1));
    angle_d_alpha_beta = atan2(d_alpha_beta(2), d_alpha_beta(1));
    phi_alpha_beta = wrapToPi(angle_n_alpha - angle_d_alpha_beta);
end