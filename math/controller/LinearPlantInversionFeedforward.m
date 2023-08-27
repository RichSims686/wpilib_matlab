classdef LinearPlantInversionFeedforward

    properties
        r;    % The current reference state.
        uff;  % The computed feedforward.
        B;
        A;
    end

    methods
        % Constructs a feedforward with the given plant.
        %
        % @param plant The plant being controlled.
        % @param dtSeconds Discretization timestep.

        % Constructs a feedforward with the given coefficients.
        %
        % @param A Continuous system matrix of the plant being controlled.
        % @param B Continuous input matrix of the plant being controlled.
        % @param dtSeconds Discretization timestep.
        
        function this = LinearPlantInversionFeedforward(varargin)
            if nargin == 2
                plant = varargin{1};
                dtSeconds = varargin{2};
                A = plant.getA();
                B = plant.getB();
            elseif nargin == 3
                A = varargin{1};
                B = varargin{2};
                dtSeconds = varargin{3};
            else
                error('Incorrect numuber of input arguments');
            end

            [this.A, this.B] = discretizeAB(A, B, dtSeconds);
            this.r = zeros(size(B,1),1);
            this.uff = zeros(size(B,2),1);
            
            this.reset();
        end
        
        
        % Returns the previously calculated feedforward as an input vector.
        %
        % @return The calculated feedforward.
        
        % Returns an element of the previously calculated feedforward.
        %
        % @param row Row of uff.
        % @return The row of the calculated feedforward.
        
        function rv = getUff(this, row)
            if nargin == 1
                rv = this.uff;
            else
                rv = this.uff(row+1, 0+1);
            end
        end
                
        
        % Returns the current reference vector r.
        %
        % @return The current reference vector.
        
        % Returns an element of the current reference vector r.
        %
        % @param row Row of r.
        % @return The row of the current reference vector.
        
        function rv = getR(this, row) 
            if nargin == 1
                rv = this.r;
            else
                rv = this.r(row+1, 0+1);
            end
        end
        
        
        % Resets the feedforward with a specified initial state vector.
        %
        % @param initialState The initial state vector.
        
        function reset(this, initialState) 
            if nargin == 1
                initialState = zeros(size(this.r));
            end
            this.r = initialState;
            this.uff(:) = 0;
        end
        
        % Calculate the feedforward with only the desired future reference. This uses the internally
        % stored "current" reference.
        %
        % <p>If this method is used the initial state of the system is the one set using @link
        % LinearPlantInversionFeedforward#reset(Matrix)end. If the initial state is not set it defaults to
        % a zero vector.
        %
        % @param nextR The reference state of the future timestep (k + dt).
        % @return The calculated feedforward.
        
        % Calculate the feedforward with current and future reference vectors.
        %
        % @param r The reference state of the current timestep (k).
        % @param nextR The reference state of the future timestep (k + dt).
        % @return The calculated feedforward.
        
        function rv = calculate(this, varargin) 
            if nargin == 1
                nextR = varargin{1};
                rv = this.calculate(this.r, nextR);
            else
                r = varargin{1}; %#ok<*PROPLC> 
                nextR = varargin{2};

                this.uff = this.B \ (nextR - (this.A * r));
                
                this.r = nextR;
                rv = this.uff;
            end
        end
    end
end
    