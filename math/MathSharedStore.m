classdef MathSharedStore
    methods (Static)
        function rv = getTimestamp()
            rv = Timestamp.get();
        end
    end
end