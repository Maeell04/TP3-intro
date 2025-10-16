function [conf] = IK_RX90(p,R,q_k)

    % p : End-effector position
    % R : End-effector orientation
    % q_k : Current configuration

    % There are n solutions to the IK of an RX90
    % We will store them in nx6 array (one solution per line)

    % Get RX90 data (length and DH params)
    [L2,L3,L6,dh] = RX90data;
    
    %%% Temporary %%%
    n = 1;
    q = zeros(n,6);
    k = 1;
    %%% End temporary %%%
    
    %%% Your code %%%
    %% ???
    %%% End of your code %%%
    
    conf = q(k,:)';

end