classdef Optional < handle
    properties
        value
    end

    methods (Static)
        function obj = empty()
            obj = Optional([]);
        end

        function obj = of(value)
            obj = Optional(value);
        end
    end

    methods
        function this = Optional(value)
            this.value = value;
        end

        function rv = isPresent(this)
            rv = ~isempty(this.value);
        end

        function rv = get(this)
            if isempty(this.value)
                throw(MException('NoSuchElementException'));
            end
            rv = this.value;
        end  
    end
end
