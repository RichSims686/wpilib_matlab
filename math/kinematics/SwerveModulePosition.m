classdef SwerveModulePosition < handle

    properties
        %* Distance measured by the wheel of the module. */
        distanceMeters = 0;
        
        %* Angle of the module. */
        angle = Rotation2d.fromDegrees(0);
    end

    methods
        %* Constructs a SwerveModulePosition with zeros for distance and angle. */

        %*
        % Constructs a SwerveModulePosition.
        %
        % @param distanceMeters The distance measured by the wheel of the module.
        % @param angle The angle of the module.
        %/

        function this = SwerveModulePosition(distanceMeters, angle)
            if nargin == 0
                this.distanceMeters = 0;
                this.angle = Rotation2d.fromDegrees(0);
            elseif nargin == 2
                this.distanceMeters = distanceMeters;
                this.angle = angle;
            end
        end
        
        function rv = equals(this, other) 
            rv = false;
            if isa(other, 'SwerveModulePosition') 
              rv = abs(other.distanceMeters - this.distanceMeters) < 1E-9 && this.angle.equals(other.angle);
            end
        end
        
        %*
        % Compares two swerve module positions. One swerve module is "greater" than the other if its
        % distance is higher than the other.
        %
        % @param other The other swerve module.
        % @rv = 1 if this is greater, 0 if both are equal, -1 if other is greater.
        %/
        function rv = compareTo(this, other) 
            rv = sign(this.distanceMeters - other.distanceMeters);
        end
        
        function rv = toString(this) 
            rv = sprintf("SwerveModulePosition(Distance: %.2f m, Angle: %s)", this.distanceMeters, this.angle);
        end
    end
end
