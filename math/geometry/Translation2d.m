classdef Translation2d < handle

%*
 % Represents a translation in 2D space. This object can be used to represent a point or a vector.
 %
 % <p>This assumes that you are using conventional mathematical axes. When the robot is at the
 % origin facing in the positive X direction, forward is positive X and left is positive Y.
 %/

    properties
        x;
        y;
    end

    methods 

        %* Constructs a Translation2d with X and Y components equal to zero. %/
        
        %*
        % Constructs a Translation2d with the X and Y components equal to the provided values.
        %
        % @param x The x component of the translation.
        % @param y The y component of the translation.
        %/
        
        %*
        % Constructs a Translation2d with the provided distance and angle. This is essentially converting
        % from polar coordinates to Cartesian coordinates.
        %
        % @param distance The distance from the origin to the end of the translation.
        % @param angle The angle between the x-axis and the translation vector.
        %/

        function this = Translation2d(varargin)
            if nargin == 0
                this.x = 0;
                this.y = 0;
            elseif nargin == 2
                if isnumeric(varargin{2})
                    this.x = varargin{1};
                    this.y = varargin{2};
                else
                    distance = varargin{1};
                    angle = varargin{2};
                    this.x = distance * angle.getCos();
                    this.y = distance * angle.getSin();                
                end
            end
        end

        %*
        % Calculates the distance between two translations in 2D space.
        %
        % <p>The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
        %
        % @param other The translation to compute the distance to.
        % @rv = The distance between the two translations.
        %/
        function rv = getDistance(this, other) 
            rv = hypot(other.x - this.x, other.y - this.y);
        end
        
        %*
        % Returns the X component of the translation.
        %
        % @rv = The X component of the translation.
        %/
        function rv = getX(this) 
            rv = this.x;
        end
        
        %*
        % Returns the Y component of the translation.
        %
        % @rv = The Y component of the translation.
        %/
        function rv = getY(this) 
            rv = this.y;
        end
        
        %*
        % Returns the norm, or distance from the origin to the translation.
        %
        % @rv = The norm of the translation.
        %/
        function rv = getNorm(this) 
        rv = hypot(this.x, this.y);
        end
        
        %*
        % Returns the angle this translation forms with the positive X axis.
        %
        % @rv = The angle of the translation
        %/
        function rv = getAngle(this) 
            rv = Rotation2d(this.x, this.y);
        end
        
        %*
        % Applies a rotation to the translation in 2D space.
        %
        % <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
        % angle.
        %
        % <pre>
        % [x_new]   [other.cos, -other.sin][x]
        % [y_new] = [other.sin,  other.cos][y]
        % </pre>
        %
        % <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will rv = a
        % Translation2d of &lt;0, 2&gt;.
        %
        % @param other The rotation to rotate the translation by.
        % @rv = The rotated translation.
        %/
        function rv = rotateBy(this, other) 
            rv = Translation2d(this.x * other.getCos() - this.y * other.getSin(), this.x * other.getSin() + this.y * other.getCos());
        end
        
        %*
        % Returns the sum of two translations in 2D space.
        %
        % <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d3.0, 8.0).
        %
        % @param other The translation to add.
        % @rv = The sum of the translations.
        %/
        function rv = plus(this, other) 
            rv = Translation2d(this.x + other.x, this.y + other.y);
        end
        
        %*
        % Returns the difference between two translations.
        %
        % <p>For example, Translation2d(5.0, 4.0) - Translation2d(1.0, 2.0) = Translation2d(4.0, 2.0).
        %
        % @param other The translation to subtract.
        % @rv = The difference between the two translations.
        %/
        function rv = minus(this, other) 
            rv = Translation2d(this.x - other.x, this.y - other.y);
        end
        
        %*
        % Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
        % flipping the point over both axes, or negating all components of the translation.
        %
        % @rv = The inverse of the current translation.
        %/
        function rv = unaryMinus(this) 
            rv = Translation2d(-this.x, -this.y);
        end
        
        %*
        % Returns the translation multiplied by a scalar.
        %
        % <p>For example, Translation2d(2.0, 2.5) % 2 = Translation2d(4.0, 5.0).
        %
        % @param scalar The scalar to multiply by.
        % @rv = The scaled translation.
        %/
        function rv = times(this, scalar) 
            rv = Translation2d(this.x * scalar, this.y * scalar);
        end
        
        %*
        % Returns the translation divided by a scalar.
        %
        % <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
        %
        % @param scalar The scalar to multiply by.
        % @rv = The reference to the mutated object.
        %/
        function rv = div(this, scalar) 
            rv = Translation2d(this.x / scalar, this.y / scalar);
        end
        
        %*
        % Returns the nearest Translation2d from a list of translations.
        %
        % @param translations The list of translations.
        % @rv = The nearest Translation2d from the list.
        %/
        function rv = nearest(this, translations)
            dx = [translation.x] - this.x;
            dy = [translation.y] - this.y;
            dist = hypot(dx, dy);
            [~,idx] = min(dist);
            rv = translations(idx);
        end
        
        function rv = toString(this) 
            rv = sprintf("Translation2d(X: %.2f, Y: %.2f)", this.x, this.y);
        end
        
        %*
        % Checks equality between this Translation2d and another object.
        %
        % @param obj The other object.
        % @rv = Whether the two objects are equal or not.
        %/
        function rv = equals(this, other)
            rv = false;
            if isa(other, 'Translation2d') 
                rv = abs(other.x - this.x) < 1E-9 && abs(other.y - this.y) < 1E-9;
            end
        end
        
        function rv = interpolate(this, endValue, t) 
            rv = Translation2d(...
                MathUtil.interpolate(this.getX(), endValue.getX(), t),...
                MathUtil.interpolate(this.getY(), endValue.getY(), t));
        end
    end
end
    