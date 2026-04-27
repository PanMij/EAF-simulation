function fis = build_varimp_fis()
    fis = mamfis( ...
        'Name', 'VarImpSupervisor', ...
        'AndMethod', 'min', ...
        'OrMethod', 'max', ...
        'ImplicationMethod', 'min', ...
        'AggregationMethod', 'max', ...
        'DefuzzificationMethod', 'centroid');

    % Input 1: normalized FSI
    fis = addInput(fis, [0 1], 'Name', 'FSI_n');
    fis = addMF(fis, 'FSI_n', 'trimf', [0 0 0.7], 'Name', 'small');
    fis = addMF(fis, 'FSI_n', 'trimf', [0.5 0.7 1], 'Name', 'medium');
    fis = addMF(fis, 'FSI_n', 'trimf', [0.7 1 1], 'Name', 'large');

    % Input 2: normalized SampEn
    fis = addInput(fis, [0 1], 'Name', 'SampEn_n');
    fis = addMF(fis, 'SampEn_n', 'trimf', [0 0 0.3], 'Name', 'small');
    fis = addMF(fis, 'SampEn_n', 'trimf', [0 0.3 0.5], 'Name', 'medium');
    fis = addMF(fis, 'SampEn_n', 'trimf', [0.3 1 1], 'Name', 'large');

    % Output: delta = DeltaZ / Zinit
    fis = addOutput(fis, [-0.03 0.01], 'Name', 'delta');
    fis = addMF(fis, 'delta', 'trimf', [-0.03  -0.03  -0.015], 'Name', 'NB');
    fis = addMF(fis, 'delta', 'trimf', [-0.025 -0.01   0],     'Name', 'NS');
    fis = addMF(fis, 'delta', 'trimf', [-0.005  0      0.005], 'Name', 'ZO');
    fis = addMF(fis, 'delta', 'trimf', [ 0      0.005  0.01],  'Name', 'PS');

    % Rule indices:
    % Inputs: small=1, medium=2, large=3
    % Output: NB=1, NS=2, ZO=3, PS=4
    %
    % Rule format:
    % [FSI_index SampEn_index output_index weight operator]
    % operator = 1 means AND

    rules = [ ...
        1 1 2 1 1  % F small,  S small  -> NS
        1 2 1 1 1  % F small,  S medium -> NB
        1 3 1 1 1  % F small,  S large  -> NB
        2 1 3 1 1  % F medium, S small  -> ZO
        2 2 2 1 1  % F medium, S medium -> NS
        2 3 2 1 1  % F medium, S large  -> NS
        3 1 4 1 1  % F large,  S small  -> PS
        3 2 3 1 1  % F large,  S medium -> ZO
        3 3 3 1 1  % F large,  S large  -> ZO
    ];

    fis = addRule(fis, rules);
end