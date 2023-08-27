classdef MathUtil
    methods (Static)

        %*
        % Returns value clamped between low and high boundaries.
        %
        % @param value Value to clamp.
        % @param low The lower boundary to which to clamp value.
        % @param high The higher boundary to which to clamp value.
        % @rv = The clamped value.
        %/        
        function rv = clamp(value, low, high)
            rv = max(low, min(value, high));
        end

        %*
        % Returns 0.0 if the given value is within the specified range around zero. The remaining range
        % between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
        %
        % @param value Value to clip.
        % @param deadband Range around zero.
        % @param maxMagnitude The maximum magnitude of the input. Can be infinite.
        % @rv = The value after the deadband is applied.
        %/

        %*
        % Returns 0.0 if the given value is within the specified range around zero. The remaining range
        % between the deadband and 1.0 is scaled from 0.0 to 1.0.
        %
        % @param value Value to clip.
        % @param deadband Range around zero.
        % @rv = The value after the deadband is applied.
        %/

        function rv = applyDeadband(value, deadband, maxMagnitude) 
            if nargin == 2
                maxMagnitude = 1;
            end

            if (abs(value) > deadband) 
                if (maxMagnitude / deadband > 1.0e12) 
                    % If max magnitude is sufficiently large, the implementation encounters
                    % roundoff error.  Implementing the limiting behavior directly avoids
                    % the problem.
                    if value > 0.0
                        rv = value - deadband;
                    else
                        rv = value + deadband;
                    end
                    return
                end
                if (value > 0.0) 
                    % Map deadband to 0 and map max to max.
                    %
                    % y - y₁ = m(x - x₁)
                    % y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                    % y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                    %
                    % (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                    % x₁ = deadband
                    % y₁ = 0
                    % x₂ = max
                    % y₂ = max
                    %
                    % y = (max - 0)/(max - deadband) (x - deadband) + 0
                    % y = max/(max - deadband) (x - deadband)
                    % y = max (x - deadband)/(max - deadband)
                    rv = maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
                    return
                else 
                    % Map -deadband to 0 and map -max to -max.
                    %
                    % y - y₁ = m(x - x₁)
                    % y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                    % y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                    %
                    % (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                    % x₁ = -deadband
                    % y₁ = 0
                    % x₂ = -max
                    % y₂ = -max
                    %
                    % y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                    % y = max/(max - deadband) (x + deadband)
                    % y = max (x + deadband)/(max - deadband)
                    rv = maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
                    return
                end
            else 
                rv = 0.0;
            end 
        end
        
        
        %*
        % Returns modulus of input.
        %
        % @param input Input value to wrap.
        % @param minimumInput The minimum value expected from the input.
        % @param maximumInput The maximum value expected from the input.
        % @rv = The wrapped value.
        %/
        function rv = inputModulus(input, minimumInput, maximumInput) 
            modulus = maximumInput - minimumInput;
            
            % Wrap input if it's above the maximum input
            numMax = floor((input - minimumInput) / modulus);
            input = input - numMax * modulus;
            
            % Wrap input if it's below the minimum input
            numMin = floor((maximumInput - input) / modulus);
            input = input + numMin * modulus;
            
            rv = input;
        end
        
        %*
        % Wraps an angle to the range -pi to pi radians.
        %
        % @param angleRadians Angle to wrap in radians.
        % @rv = The wrapped angle.
        %/
        function rv = angleModulus(angleRadians) 
            rv = MathUtil.inputModulus(angleRadians, -pi, pi);
        end

        % Perform linear interpolation between two values.
        %
        % @param startValue The value to start at.
        % @param endValue The value to end at.
        % @param t How far between the two values to interpolate. This is clamped to [0, 1].
        % @rv = The interpolated value.
        function rv = interpolate(startValue, endValue, t) 
            rv = startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1);
        end        
    end
end