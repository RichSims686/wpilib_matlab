classdef TimeInterpolatableBuffer < handle

    %*
    % The TimeInterpolatableBuffer provides an easy way to estimate past measurements. One application
    % might be in conjunction with the DifferentialDrivePoseEstimator, where knowledge of the robot
    % pose at the time when vision or other global measurement were recorded is necessary, or for
    % recording the past angles of mechanisms as measured by encoders.
    %
    % @param <T> The type stored in this buffer.
    %/

    properties  
        historySizeSeconds;
        interpolatingFunc;
        pastSnapshots;
    end

    methods (Static)
        %*
        % Create a new TimeInterpolatableBuffer to store Double values.
        %
        % @param historySizeSeconds The history size of the buffer.
        % @rv = The new TimeInterpolatableBuffer.
        %/
        function obj = createBuffer(interpolateFunction, historySizeSeconds)            
            obj = TimeInterpolatableBuffer(interpolateFunction, historySizeSeconds);
        end
    end         

    methods
        function this = TimeInterpolatableBuffer(interpolateFunction, historySizeSeconds) 
            this.interpolatingFunc = interpolateFunction;
            this.historySizeSeconds = historySizeSeconds;
            this.pastSnapshots = containers.Map('KeyType', 'double', 'ValueType', 'any');
        end

        
        %*
        % Add a sample to the buffer.
        %
        % @param timeSeconds The timestamp of the sample.
        % @param sample The sample object.
        %/
        function addSample(this, timeSeconds, sample) 
            this.cleanUp(timeSeconds);
            this.pastSnapshots(timeSeconds) = sample;
        end
        
        %*
        % Removes samples older than our current history size.
        %
        % @param time The current timestamp.
        %/
        function cleanUp(this, time)
            keySet = keys(this.pastSnapshots);
            if ~isempty(keySet)
                removeFlag = (time - [keySet{:}]) >= this.historySizeSeconds;
                if any(removeFlag)
                    remove(this.pastSnapshots, keySet(removeFlag));
                end
            end
        end
        
        %* Clear all old samples.%/
        function clear(this) 
            remove(this.pastSnapshots, keys(this.pastSnapshots));
        end
        
        %*
        % Sample the buffer at the given time. If the buffer is empty, an empty Optional is returned.
        %
        % @param timeSeconds The time at which to sample.
        % @rv = The interpolated value at that timestamp or an empty Optional.
        %/
        function rv = getSample(this, timeSeconds) 
            if isempty(this.pastSnapshots) 
                rv = Optional.empty();
                return
            end
            
            % Special case for when the requested time is the same as a sample
            if isKey(this.pastSnapshots, timeSeconds)
                nowEntry = this.pastSnapshots(timeSeconds);
                rv = Optional.of(nowEntry);
                return
            end

            keySet = keys(this.pastSnapshots);
            topKey = min(keySet(keySet >= timeSeconds));
            bottomKey = max(keySet(keySet <= timeSeconds));

            % return null if neither sample exists, and the opposite bound if the other is null
            if isempty(topKey) && isempty(bottomKey) 
                rv = Optional.empty();
                return
            elseif isempty(topKey) 
                rv = Optional.of(this.pastSnapshots(bottomKey));
                return
            elseif isempty(bottomKey) 
                rv = Optional.of(this.pastSnapshots(topKey));
                return
            else 
                % Otherwise, interpolate. Because T is between [0, 1], we want the ratio of (the difference
                % between the current time and bottom bound) and (the difference between top and bottom
                % bounds).
                rv = Optional.of(...
                    this.interpolatingFunc.interpolate(...
                        this.pastSnapshots(bottomKey),...
                        this.pastSnapshots(topKey),...
                        (timeSeconds - bottomKey) / (topKey - bottomKey)));
            end
        end
        
        %*
        % Grant access to the internal sample buffer. Used in Pose Estimation to replay odometry inputs
        % stored within this buffer.
        %
        % @rv = The internal sample buffer.
        %/
        function rv = getInternalBuffer(this) 
            rv = this.pastSnapshots;
        end
        
        function rv = getInternalBufferTailMap(this, timeSeconds) 
            keySet = keys(this.pastSnapshots);
            rv = this.pastSnapshots(keySet(keySet >= timeSeconds));
        end
        
        function rv = getInternalBufferLastKey(this) 
            rv = max(keys(this.pastSnapshots));
        end
        
    end
end
