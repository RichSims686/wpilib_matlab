classdef StateSpaceUtil
    methods (Static)


        %*
        % Creates a vector of normally distributed white noise with the given noise intensities for each
        % element.
        %
        % @param <N> Num representing the dimensionality of the noise vector to create.
        % @param stdDevs A matrix whose elements are the standard deviations of each element of the noise
        %     vector.
        % @return White noise vector.
        %/
        function rv = makeWhiteNoiseVector(stdDevs)
            rv = stdDevs .* randn(size(stdDevs));       
        end



        %*
        % Renormalize all inputs if any exceeds the maximum magnitude. Useful for systems such as
        % differential drivetrains.
        %
        % @param u The input vector.
        % @param maxMagnitude The maximum magnitude any input can have.
        % @param <I> The number of inputs.
        % @return The normalizedInput
        %/        
        function rv = desaturateInputVector(u, maxMagnitude)
            maxInput = max(abs(u));
            isCapped = maxInput > maxMagnitude;

            if isCapped
                rv = u * (maxMagnitude / maxInput);
            else
                rv = u;
            end
        end
    end
end