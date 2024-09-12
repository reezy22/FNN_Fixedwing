function y = FLNN_Controller_2(PIDctrl, e, edot)
    %#codegen

    % Step 1: Initialize Fuzzy Logic Controller
    persistent fis net;
    
    if isempty(fis) || isempty(net)
        % Define the Fuzzy Inference System (FIS)
        fis = mamfis('Name', 'UAV_FLNN_Controller');

        % Define the input variables (Error and Error Dot)
        fis = addInput(fis, [-1 1], 'Name', 'Error');
        fis = addInput(fis, [-1 1], 'Name', 'ErrorDot');

        % Define the output variable (Correction to be added to PID output)
        fis = addOutput(fis, [-1 1], 'Name', 'Correction');

        % Define the membership functions for Error
        fis = addMF(fis, 'Error', 'gaussmf', [0.2 0], 'Name', 'Zero');
        fis = addMF(fis, 'Error', 'gaussmf', [0.2 0.5], 'Name', 'Positive');
        fis = addMF(fis, 'Error', 'gaussmf', [0.2 -0.5], 'Name', 'Negative');

        % Define the membership functions for ErrorDot
        fis = addMF(fis, 'ErrorDot', 'gaussmf', [0.2 0], 'Name', 'Zero');
        fis = addMF(fis, 'ErrorDot', 'gaussmf', [0.2 0.5], 'Name', 'Positive');
        fis = addMF(fis, 'ErrorDot', 'gaussmf', [0.2 -0.5], 'Name', 'Negative');

        % Define the membership functions for Correction
        fis = addMF(fis, 'Correction', 'gaussmf', [0.2 0], 'Name', 'Zero');
        fis = addMF(fis, 'Correction', 'gaussmf', [0.2 0.5], 'Name', 'Positive');
        fis = addMF(fis, 'Correction', 'gaussmf', [0.2 -0.5], 'Name', 'Negative');

        % Define fuzzy rules
        ruleList = [
            1 1 1 1 1;
            1 2 2 1 1;
            1 3 3 1 1;
            2 1 2 1 1;
            2 2 3 1 1;
            2 3 3 1 1;
            3 1 3 1 1;
            3 2 3 1 1;
            3 3 3 1 1;
        ];

        fis = addRule(fis, ruleList);

        % Step 2: Define and train the Neural Network (Load or initialize)
        inputData = randn(100, 2); % Replace with actual UAV data
        desiredOutput = randn(100, 1); % Replace with actual UAV data

        net = feedforwardnet(10); % 10 hidden neurons
        net = train(net, inputData', desiredOutput');
    end

    % Step 3: Simulate the Fuzzy Logic Neural Network Controller
    % Combine input signals into a vector
    sampleInput = [e, edot];

    % Use the fuzzy logic system to compute the initial output
    fuzzyOutput = evalfis(fis, sampleInput);

    % Use the neural network to refine the output
    refinedOutput = net(sampleInput');

    % Combine fuzzy and neural network output
    correction = 0.7 * fuzzyOutput + 0.3 * refinedOutput;

    % Combine the correction with the PID control output
    y = PIDctrl + correction;
end