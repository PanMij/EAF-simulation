function [Zref, Zadj, dZ, delta] = varimp_supervisor_step(FSI_n, SampEn_n, Zinit, Zadj_prev, fis)
% FSI_n, SampEn_n, Zinit, Zadj_prev are all 3x1 vectors

    FSI_n     = reshape(FSI_n, 3, 1);
    SampEn_n  = reshape(SampEn_n, 3, 1);
    Zinit     = reshape(Zinit, 3, 1);
    Zadj_prev = reshape(Zadj_prev, 3, 1);

    Zref  = zeros(3,1);
    Zadj  = zeros(3,1);
    dZ    = zeros(3,1);
    delta = zeros(3,1);

    for p = 1:3
        % Fuzzy inference: delta = DeltaZ / Zinit
        delta(p) = evalfis(fis, [FSI_n(p), SampEn_n(p)]);

        % Candidate impedance increment
        dZ_star = delta(p) * Zinit(p);

        % Candidate adjustable impedance
        Zcand = Zadj_prev(p) + dZ_star;

        % Bounds
        Zadj_min = -0.1 * Zinit(p);
        Zadj_max = 0.0;

        % Anti-windup-style freeze
        if (Zcand < Zadj_min) || (Zcand > Zadj_max)
            Zadj(p) = Zadj_prev(p);
            dZ(p)   = 0.0;
        else
            Zadj(p) = Zcand;
            dZ(p)   = dZ_star;
        end

        % Final reference
        Zref(p) = Zinit(p) + Zadj(p);
    end
end