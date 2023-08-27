classdef Timestamp < handle
    methods (Access=private, Static)
        % this stupid function is needed to implement a static variable
        function rv = getSet(newTimestamp)
            persistent timestamp;
            if nargin
                timestamp = newTimestamp;
            end
            rv = timestamp;

            if isempty(rv)
                error('Timestamp object not instantiated');
            end
        end
    end

    methods (Access=public, Static)
        function rv = set(value)
            rv = Timestamp.getSet(value);
        end

        function rv = get()
            rv = Timestamp.getSet();
        end

        function rv = increment(dtSeconds)
            rv = Timestamp.set( Timestamp.get() + dtSeconds );
        end

        function rv = getFPGATimestamp()
            rv = floor(Timestamp.get() * 1e6);
        end
    end

    methods
        function this = Timestamp()
            Timestamp.set(0);
        end
    end
end