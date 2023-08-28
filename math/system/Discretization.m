classdef Discretization
    
    methods (Static)

        % Discretizes the given continuous A and B matrices.
        %
        % @param <States> Nat representing the states of the system.
        % @param <Inputs> Nat representing the inputs to the system.
        % @param contA Continuous system matrix.
        % @param contB Continuous input matrix.
        % @param dtSeconds Discretization timestep.
        % @return a Pair representing discA and discB.

        function [Ad, Bd] = discretizeAB(A, B, dtSeconds)
            sysc = ss(A,B,1,0);             % continuous system
            sysd = c2d(sysc, dtSeconds);    % discrete system
            Ad = sysd.A;
            Bd = sysd.B;
        end
    end
end