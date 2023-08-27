classdef SwerveModuleState < handle

    properties        
        speedMetersPerSecond;   %* Speed of the wheel of the module. */
        angle;                  %* Angle of the module. */
    end

    methods    
        %* Constructs a SwerveModuleState with zeros for speed and angle. */

        %*
        % Constructs a SwerveModuleState.
        %
        % @param speedMetersPerSecond The speed of the wheel of the module.
        % @param angle The angle of the module.
        %/

        function this = SwerveModuleState(speedMetersPerSecond, angle)
            if nargin == 0
                this.speedMetersPerSecond = 0;           %* Speed of the wheel of the module. */
                this.angle = Rotation2d.fromDegrees(0);  %* Angle of the module. */
            elseif nargin == 2
                this.speedMetersPerSecond = speedMetersPerSecond;
                this.angle = angle;
            end
        end

        function rv = equals(this, other) 
            if isa(obj, 'SwerveModuleState') 
              rv = abs(other.speedMetersPerSecond - this.speedMetersPerSecond) < 1E-9 ...
                  && this.angle.equals(other.angle);
            else
                rv = false;
            end
        end
        
        %*
        % Compares two swerve module states. One swerve module is "greater" than the other if its speed
        % is higher than the other.
        %
        % @param other The other swerve module.
        % @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
        %/
        function rv = compareTo(this, other) 
            rv = sign(this.speedMetersPerSecond - other.speedMetersPerSecond);
        end
        
        function rv =  toString(this) 
            rv = sprintf("SwerveModuleState(Speed: %.2f m/s, Angle: %s)", this.speedMetersPerSecond, this.angle);
        end
        
    end

    methods (Static)
        %*
        % Minimize the change in heading the desired swerve module state would require by potentially
        % reversing the direction the wheel spins. If this is used with the PIDController class's
        % continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
        %
        % @param desiredState The desired state.
        % @param currentAngle The current module angle.
        % @return Optimized swerve module state.
        %/
        function rv = optimize(desiredState, currentAngle) 
            delta = desiredState.angle.minus(currentAngle);
            if (abs(delta.getDegrees()) > 90.0) 
                rv = SwerveModuleState(...
                    -desiredState.speedMetersPerSecond, ...
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
            else 
                rv = SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
            end
        end
    end
end
