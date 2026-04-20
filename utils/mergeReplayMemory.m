function mergedBuf = mergeReplayMemory(buf1, buf2, maxLength)
%MERGEREPLAYMEMORY Merge two rlReplayMemory objects into one.
%
% mergedBuf = mergeReplayMemory(buf1, buf2)
% mergedBuf = mergeReplayMemory(buf1, buf2, maxLength)

    arguments
        buf1 (1,1) rl.replay.rlReplayMemory
        buf2 (1,1) rl.replay.rlReplayMemory
        maxLength (1,1) double {mustBeInteger,mustBePositive} = []
    end

    % Check compatibility
    obs1 = getObservationInfo(buf1);
    obs2 = getObservationInfo(buf2);
    act1 = getActionInfo(buf1);
    act2 = getActionInfo(buf2);

    if ~localSpecEqual(obs1, obs2)
        error("Observation specs of buf1 and buf2 are not compatible.");
    end
    if ~localSpecEqual(act1, act2)
        error("Action specs of buf1 and buf2 are not compatible.");
    end

    % Choose capacity
    if isempty(maxLength)
        maxLength = buf1.Length + buf2.Length;
    end

    % Create destination buffer
    mergedBuf = rlReplayMemory(obs1, act1, maxLength);

    % Extract all experiences
    exp1 = allExperiences(buf1);
    exp2 = allExperiences(buf2);

    % Append buffer 1
    if ~isempty(exp1)
        validateExperience(mergedBuf, exp1);
        append(mergedBuf, exp1);
    end

    % Append buffer 2
    if ~isempty(exp2)
        validateExperience(mergedBuf, exp2);
        append(mergedBuf, exp2);
    end
end

function tf = localSpecEqual(a, b)
% Compare spec arrays conservatively by class, size, and key properties.
    if numel(a) ~= numel(b)
        tf = false;
        return;
    end

    tf = true;
    for k = 1:numel(a)
        if ~strcmp(class(a(k)), class(b(k)))
            tf = false; return;
        end

        % Compare common properties that matter for replay compatibility
        if isprop(a(k),"Dimension") && ~isequal(a(k).Dimension, b(k).Dimension)
            tf = false; return;
        end
        if isprop(a(k),"LowerLimit") && ~isequaln(a(k).LowerLimit, b(k).LowerLimit)
            tf = false; return;
        end
        if isprop(a(k),"UpperLimit") && ~isequaln(a(k).UpperLimit, b(k).UpperLimit)
            tf = false; return;
        end
        if isprop(a(k),"Elements") && ~isequal(a(k).Elements, b(k).Elements)
            tf = false; return;
        end
    end
end