classdef PIDController < handle

    properties
        
        kp; % Factor for "proportional" control
        ki; % Factor for "integral" control
        kd; % Factor for "derivative" control
        
        % The period (in seconds) of the loop that calls the controller
        period;
        
        maximumIntegral = 1.0;
        minimumIntegral = -1.0;
        
        maximumInput;
        minimumInput;
        
        % Do the endpoints wrap around? e.g. Absolute encoder
        continuous;
        
        % The error at the time of the most recent call to calculate()
        positionError = 0;
        velocityError = 0;
        
        % The error at the time of the second-most-recent call to calculate() (used to compute velocity)
        prevError = 0;
        
        % The sum of the errors for use in the integral calc
        totalError = 0;
        
        % The error that is considered at setpoint.
        positionTolerance = 0.05;
        velocityTolerance = Inf;
        
        setpoint = 0;
        measurement = 0;
        
        haveMeasurement = false;
        haveSetpoint = false;
    end

    methods
        %*
        % Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
        % 0.02 seconds.
        %
        % @param kp The proportional coefficient.
        % @param ki The integral coefficient.
        % @param kd The derivative coefficient.
        %/
        %*
        % Allocates a PIDController with the given constants for kp, ki, and kd.
        %
        % @param kp The proportional coefficient.
        % @param ki The integral coefficient.
        % @param kd The derivative coefficient.
        % @param period The period between controller updates in seconds. Must be non-zero and positive.
        %/
        function this = PIDController(kp, ki, kd, period)
            if nargin<4
                period = 0.02;
            end
    
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            
            if (period <= 0) 
              throw MException(IllegalArgumentException', "Controller period must be a non-zero positive number!");
            end
            this.period = period;
        end
    
        
        function close(this)  %#ok<MANU> 
        end
        
        %*
        % Sets the PID Controller gain parameters.
        %
        % <p>Set the proportional, integral, and differential coefficients.
        %
        % @param kp The proportional coefficient.
        % @param ki The integral coefficient.
        % @param kd The derivative coefficient.
        %/
        function setPID(this, kp, ki, kd) 
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        end
        
        %*
        % Sets the Proportional coefficient of the PID controller gain.
        %
        % @param kp proportional coefficient
        %/
        function setP(this, kp) 
            this.kp = kp;
        end
        
        %*
        % Sets the Integral coefficient of the PID controller gain.
        %
        % @param ki integral coefficient
        %/
        function setI(this, ki) 
            this.ki = ki;
        end
        
        %*
        % Sets the Differential coefficient of the PID controller gain.
        %
        % @param kd differential coefficient
        %/
        function setD(this, kd) 
            this.kd = kd;
        end
        
        %*
        % Get the Proportional coefficient.
        %
        % @rv = proportional coefficient
        %/
        function rv = getP(this) 
            rv = this.kp;
        end
        
        %*
        % Get the Integral coefficient.
        %
        % @rv = integral coefficient
        %/
        function rv = getI(this) 
            rv = this.ki;
        end
        
        %*
        % Get the Differential coefficient.
        %
        % @rv = differential coefficient
        %/
        function rv = getD(this) 
            rv = this.kd;
        end
        
        %*
        % rv =s the period of this controller.
        %
        % @rv = the period of the controller.
        %/
        function rv = getPeriod(this) 
            rv = this.period;
        end
        
        %*
        % rv =s the position tolerance of this controller.
        %
        % @rv = the position tolerance of the controller.
        %/
        function rv = getPositionTolerance(this) 
            rv = this.positionTolerance;
        end
        
        %*
        % rv =s the velocity tolerance of this controller.
        %
        % @rv = the velocity tolerance of the controller.
        %/
        function rv = getVelocityTolerance(this) 
            rv = this.velocityTolerance;
        end
        
        %*
        % Sets the setpoint for the PIDController.
        %
        % @param setpoint The desired setpoint.
        %/
        function setSetpoint(this, setpoint) 
            this.setpoint = setpoint;
            this.haveSetpoint = true;
            
            if (this.continuous) 
                errorBound = (this.maximumInput - this.minimumInput) / 2.0;
                this.positionError = MathUtil.inputModulus(this.setpoint - this.measurement, -errorBound, errorBound);
            else 
                this.positionError = this.setpoint - this.measurement;
            end
            
            this.velocityError = (this.positionError - this.prevError) / this.period;
        end
        
        %*
        % rv =s the current setpoint of the PIDController.
        %
        % @rv = The current setpoint.
        %/
        function rv = getSetpoint(this) 
            rv = this.setpoint;
        end
        
        %*
        % rv =s true if the error is within the tolerance of the setpoint.
        %
        % <p>This will rv = false until at least one input value has been computed.
        %
        % @rv = Whether the error is within the acceptable bounds.
        %/
        function rv = atSetpoint(this) 
            rv = this.haveMeasurement ...
                && this.haveSetpoint ...
                && abs(this.positionError) < this.positionTolerance ...
                && abs(this.velocityError) < this.velocityTolerance;
        end
        
        %*
        % Enables continuous input.
        %
        % <p>Rather then using the max and min input range as constraints, it considers them to be the
        % same point and automatically calculates the shortest route to the setpoint.
        %
        % @param minimumInput The minimum value expected from the input.
        % @param maximumInput The maximum value expected from the input.
        %/
        function enableContinuousInput(this, minimumInput, maximumInput) 
            this.continuous = true;
            this.minimumInput = minimumInput;
            this.maximumInput = maximumInput;
        end
        
        %* Disables continuous input. */
        function disableContinuousInput(this) 
            this.continuous = false;
        end
        
        %*
        % rv =s true if continuous input is enabled.
        %
        % @rv = True if continuous input is enabled.
        %/
        function rv = isContinuousInputEnabled(this) 
            rv = this.continuous;
        end
        
        %*
        % Sets the minimum and maximum values for the integrator.
        %
        % <p>When the cap is reached, the integrator value is added to the controller output rather than
        % the integrator value times the integral gain.
        %
        % @param minimumIntegral The minimum value of the integrator.
        % @param maximumIntegral The maximum value of the integrator.
        %/
        function setIntegratorRange(this, minimumIntegral, maximumIntegral) 
            this.minimumIntegral = minimumIntegral;
            this.maximumIntegral = maximumIntegral;
        end
        
        %*
        % Sets the error which is considered tolerable for use with atSetpoint().
        %
        % @param positionTolerance Position error which is tolerable.
        %/

                %*
        % Sets the error which is considered tolerable for use with atSetpoint().
        %
        % @param positionTolerance Position error which is tolerable.
        % @param velocityTolerance Velocity error which is tolerable.
        %/

        function setTolerance(this,  positionTolerance, velocityTolerance)

            this.positionTolerance = positionTolerance;

            if nargin == 3
                this.velocityTolerance = velocityTolerance;
            else
                this.velocityTolerance = Inf;
            end
        end
                
        %*
        % rv =s the difference between the setpoint and the measurement.
        %
        % @rv = The error.
        %/
        function rv = getPositionError(this) 
            rv = this.positionError;
        end
        
        %*
        % rv =s the velocity error.
        %
        % @rv = The velocity error.
        %/
        function rv = getVelocityError(this) 
            rv = this.velocityError;
        end
        
        %*
        % rv =s the next output of the PID controller.
        %
        % @param measurement The current measurement of the process variable.
        % @param setpoint The new setpoint of the controller.
        % @rv = The next controller output.
        %/
        
        %*
        % rv =s the next output of the PID controller.
        %
        % @param measurement The current measurement of the process variable.
        % @rv = The next controller output.
        %/
        function rv = calculate(this,  measurement, setpoint)
            if nargin == 3
                this.setpoint = setpoint;
                this.haveSetpoint = true;
            end
            this.measurement = measurement;
            this.prevError = this.positionError;
            this.haveMeasurement = true;
            
            if (this.continuous) 
                errorBound = (this.maximumInput - this.minimumInput) / 2.0;
                this.positionError = MathUtil.inputModulus(this.setpoint - this.measurement, -errorBound, errorBound);
            else 
                this.positionError = this.setpoint - this.measurement;
            end
            
            this.velocityError = (this.positionError - this.prevError) / this.period;
            
            if (this.ki ~= 0) 
                this.totalError = ...
                  MathUtil.clamp( ...
                      this.totalError + this.positionError * this.period, ...
                      this.minimumIntegral / this.ki, ...
                      this.maximumIntegral / this.ki);
            end
            
            rv = this.kp * this.positionError + this.ki * this.totalError + this.kd * this.velocityError;
        end
        
        %* Resets the previous error and the integral term. */
        function reset(this) 
            this.positionError = 0;
            this.prevError = 0;
            this.totalError = 0;
            this.velocityError = 0;
            this.haveMeasurement = false;
        end
        
    end
end
