classdef SimpleMotorFeedforward < handle

    properties
      ks;
      kv;
      ka;
    end

    methods
        % Creates a new SimpleMotorFeedforward with the specified gains. Units of the gain values will
        % dictate units of the computed feedforward.
        %
        % @param ks The static gain.
        % @param kv The velocity gain.
        % @param ka The acceleration gain.
        function this = SimpleMotorFeedforward(ks, kv, ka) 
            if nargin<3
                ka = 0;
            end
            this.ks = ks;
            this.kv = kv;
            this.ka = ka;
          end

        % Calculates the feedforward from the gains and setpoints.
        %
        % @param velocity The velocity setpoint.
        % @param acceleration The acceleration setpoint.
        % @rv = The computed feedforward.

        % Calculates the feedforward from the gains and setpoints.
        %
        % @param currentVelocity The current velocity setpoint.
        % @param nextVelocity The next velocity setpoint.
        % @param dtSeconds Time between velocity setpoints in seconds.
        % @rv = The computed feedforward.        
        


        function rv = calculate(this, varargin) 
            if nargin == 1 || nargin == 2
                velocity = varargin{1};
                acceleration = 0;
                if nargin == 3
                    acceleration = varargin{2};
                end
                rv = this.ks * sign(velocity) + this.kv * velocity + this.ka * acceleration;

            elseif nargin == 4
                currentVelocity = varargin{1};
                nextVelocity = varargin{2};
                dtSeconds = varargin{3};

                plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
                feedforward = LinearPlantInversionFeedforward(plant, dtSeconds);
                
                r = currentVelocity;
                nextR = nextVelocity;
                
                ff = feedforward.calculate(r, nextR);
                rv = this.ks * sign(currentVelocity) + ff(1);                
            else
                error('Incorrect number of input arguments');
            end
        end
        


        % Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
        % Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        % simultaneously achievable - enter the acceleration constraint, and this will give you a
        % simultaneously-achievable velocity constraint.
        %
        % @param maxVoltage The maximum voltage that can be supplied to the motor.
        % @param acceleration The acceleration of the motor.
        % @rv = The maximum possible velocity at the given acceleration.

        function rv = maxAchievableVelocity(this, maxVoltage, acceleration) 
        % Assume max velocity is positive
            rv = (maxVoltage - this.ks - acceleration * this.ka) / this.kv;
        end
        

        % Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
        % Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        % simultaneously achievable - enter the acceleration constraint, and this will give you a
        % simultaneously-achievable velocity constraint.
        %
        % @param maxVoltage The maximum voltage that can be supplied to the motor.
        % @param acceleration The acceleration of the motor.
        % @rv = The minimum possible velocity at the given acceleration.

        function rv = minAchievableVelocity(this, maxVoltage, acceleration) 
        % Assume min velocity is negative, ks flips sign
            rv = (-maxVoltage + this.ks - acceleration * this.ka) / this.kv;
        end
        

        % Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
        % Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        % simultaneously achievable - enter the velocity constraint, and this will give you a
        % simultaneously-achievable acceleration constraint.
        %
        % @param maxVoltage The maximum voltage that can be supplied to the motor.
        % @param velocity The velocity of the motor.
        % @rv = The maximum possible acceleration at the given velocity.

        function rv = maxAchievableAcceleration(this, maxVoltage, velocity) 
            rv = (maxVoltage - this.ks * sign(velocity) - velocity * this.kv) / this.ka;
        end
        

        % Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
        % Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
        % simultaneously achievable - enter the velocity constraint, and this will give you a
        % simultaneously-achievable acceleration constraint.
        %
        % @param maxVoltage The maximum voltage that can be supplied to the motor.
        % @param velocity The velocity of the motor.
        % @rv = The minimum possible acceleration at the given velocity.

        function rv = minAchievableAcceleration(this, maxVoltage, velocity) 
            rv = this.maxAchievableAcceleration(-maxVoltage, velocity);
        end
    end
end
