classdef Rotation2d
    properties
        value = 0;
        cos = 1;
        sin = 0;
    end

    methods (Static)
        %*
        % Constructs and returns a Rotation2d with the given radian value.
        %
        % @param radians The value of the angle in radians.
        % @rv = The rotation object with the desired angle value.
        %/
        function rv = fromRadians(radians) 
            rv = Rotation2d(radians);
        end
        
        %*
        % Constructs and returns a Rotation2d with the given degree value.
        %
        % @param degrees The value of the angle in degrees.
        % @rv = The rotation object with the desired angle value.
        %/
        function rv = fromDegrees(degrees) 
            rv = Rotation2d(deg2rad(degrees));
        end
        
        %*
        % Constructs and returns a Rotation2d with the given number of rotations.
        %
        % @param rotations The value of the angle in rotations.
        % @rv = The rotation object with the desired angle value.
        %/
        function rv = fromRotations(rotations) 
            rv = Rotation2d(Units.rotationsToRadians(rotations));
        end
    end


    methods
        %* Constructs a Rotation2d with a default angle of 0 degrees. */

        %*
        % Constructs a Rotation2d with the given radian value.
        %
        % @param value The value of the angle in radians.
        %/

        %*
        % Constructs a Rotation2d with the given x and y (cosine and sine) components.
        %
        % @param x The x component or cosine of the rotation.
        % @param y The y component or sine of the rotation.
        %/
        
        function this = Rotation2d(varargin)
            if nargin == 0
                this.value = 0.0;
                this.cos = 1.0;
                this.sin = 0.0;
            elseif nargin == 1
                value = varargin{1};
                this.value = value;
                this.cos = cos(value);
                this.sin = sin(value);
            elseif nargin == 2
                x = varargin{1};
                y = varargin{2};
                magnitude = hypot(x, y);
                if (magnitude > 1e-6) 
                    this.sin = y / magnitude;
                    this.cos = x / magnitude;
                else 
                    this.sin = 0.0;
                    this.cos = 1.0;
                end
                this.value = atan2(this.sin, this.cos);
            else
                error('Wrong number of input args');
            end
        end
        
        %*
        % Adds two rotations together, with the result being bounded between -pi and pi.
        %
        % <p>For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
        % <code>Rotation2d(PI/2.0)</code>
        %
        % @param other The rotation to add.
        % @rv = The sum of the two rotations.
        %/
        function rv = plus(this, other) 
            rv = this.rotateBy(other);
        end
        
        %*
        % Subtracts the new rotation from the current rotation and returns the new rotation.
        %
        % <p>For example, <code>Rotation2d.fromDegrees(10).minus(Rotation2d.fromDegrees(100))</code>
        % equals <code>Rotation2d(-PI/2.0)</code>
        %
        % @param other The rotation to subtract.
        % @rv = The difference between the two rotations.
        %/
        function rv = minus(this, other) 
            rv = this.rotateBy(other.unaryMinus());
        end
        
        %*
        % Takes the inverse of the current rotation. This is simply the negative of the current angular
        % value.
        %
        % @rv = The inverse of the current rotation.
        %/
        function rv = unaryMinus(this) 
            rv = Rotation2d(-this.value);
        end
        
        %*
        % Multiplies the current rotation by a scalar.
        %
        % @param scalar The scalar.
        % @rv = The new scaled Rotation2d.
        %/
        function rv = times(this, scalar) 
            rv = Rotation2d(this.value * scalar);
        end
        
        %*
        % Divides the current rotation by a scalar.
        %
        % @param scalar The scalar.
        % @rv = The new scaled Rotation2d.
        %/
        function rv = div(this, scalar) 
            rv = this.times(1.0 / scalar);
        end
        
        %*
        % Adds the new rotation to the current rotation using a rotation matrix.
        %
        % <p>The matrix multiplication is as follows:
        %
        % <pre>
        % [cos_new]   [other.cos, -other.sin][cos]
        % [sin_new] = [other.sin,  other.cos][sin]
        % value_new = atan2(sin_new, cos_new)
        % </pre>
        %
        % @param other The rotation to rotate by.
        % @rv = The new rotated Rotation2d.
        %/
        function rv = rotateBy(this, other) 
            rv = Rotation2d(this.cos * other.cos - this.sin * other.sin, this.cos * other.sin + this.sin * other.cos);
        end
        
        %*
        % returns the radian value of the Rotation2d.
        %
        % @rv = The radian value of the Rotation2d.
        % @see edu.wpi.first.MathUtil#angleModulus(this,) to constrain the angle within (-pi, pi]
        %/
        function rv = getRadians(this) 
            rv = this.value;
        end
        
        %*
        % returns the degree value of the Rotation2d.
        %
        % @rv = The degree value of the Rotation2d.
        % @see edu.wpi.first.MathUtil#inputModulus(this,, double, double) to constrain the angle
        %     within (-180, 180]
        %/
        function rv = getDegrees(this) 
            rv = rad2deg(this.value);
        end
        
        %*
        % returns the number of rotations of the Rotation2d.
        %
        % @rv = The number of rotations of the Rotation2d.
        %/
        function rv = getRotations(this) 
            rv = Units.radiansToRotations(this.value);
        end
        
        %*
        % returns the cosine of the Rotation2d.
        %
        % @rv = The cosine of the Rotation2d.
        %/
        function rv = getCos(this) 
            rv = this.cos;
        end
        
        %*
        % returns the sine of the Rotation2d.
        %
        % @rv = The sine of the Rotation2d.
        %/
        function rv = getSin(this) 
            rv = this.sin;
        end
        
        %*
        % returns the tangent of the Rotation2d.
        %
        % @rv = The tangent of the Rotation2d.
        %/
        function rv = getTan(this) 
            rv = this.sin / this.cos;
        end
        
        function rv = toString(this) 
            rv = sprintf("Rotation2d(Rads: %.2f, Deg: %.2f)", this.value, rad2deg(this.value));
        end
        
        %*
        % Checks equality between this Rotation2d and another object.
        %
        % @param obj The other object.
        % @rv = Whether the two objects are equal or not.
        %/
        function rv = equals(this, other) 
            rv = false;
            if isa(other, 'Rotation2d') 
                rv = hypot(this.cos - other.this.cos, this.sin - other.this.sin) < 1E-9;
            end
        end

        function rv = interpolate(this, endValue, t) 
            rv = this.plus(endValue.minus(this).times(MathUtil.clamp(t, 0, 1)));
        end
    end
end