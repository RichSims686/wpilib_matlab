classdef Transform2d < handle
    
        %* Represents a transformation for a Pose2d. */

    properties
        translation = Translation2d();
        rotation = Rotation2d();
    end
        
    methods
        %* Constructs the identity transform -- maps an initial pose to itself. */

        %*
        % Constructs a transform with the given translation and rotation components.
        %
        % @param translation Translational component of the transform.
        % @param rotation Rotational component of the transform.
        %/
        
        %*
        % Constructs the transform that maps the initial pose to the final pose.
        %
        % @param initial The initial pose for the transformation.
        % @param last The final pose for the transformation.
        %/
        function this = Transform2d(varargin) 
            if nargin == 0
                this.translation = Translation2d();
                this.rotation = Rotation2d();
            elseif nargin == 2
                if isa(varargin{1},'Translation2d')
                    this.translation = varargin{1};
                    this.rotation = varargin{2};
                elseif isa(varargin{1}, 'Pose2d')
                    initial = varargin{1};
                    last = varargin{2};
            
                    % We are rotating the difference between the translations
                    % using a clockwise rotation matrix. This transforms the global
                    % delta into a local delta (relative to the initial pose).
                    this.translation = ...
                    last.getTranslation() ...
                        .minus(initial.getTranslation()) ...
                        .rotateBy(initial.getRotation().unaryMinus());
                    
                    this.rotation = last.getRotation().minus(initial.getRotation());
                end
            end
        end
        

        %*
        % Multiplies the transform by the scalar.
        %
        % @param scalar The scalar.
        % @rv = The scaled Transform2d.
        %/
        function rv = times(this, scalar) 
            rv = Transform2d(this.translation.times(scalar), this.rotation.times(scalar));
        end
        
        %*
        % Divides the transform by the scalar.
        %
        % @param scalar The scalar.
        % @rv = The scaled Transform2d.
        %/
        function rv = div(this, scalar) 
            rv = this.times(1.0 / scalar);
        end
        
        %*
        % Composes two transformations.
        %
        % @param other The transform to compose with this one.
        % @rv = The composition of the two transformations.
        %/
        function rv = plus(this, other) 
            rv = Transform2d(Pose2d(), Pose2d().transformBy(this).transformBy(other));
        end
        
        %*
        % Returns the translation component of the transformation.
        %
        % @rv = The translational component of the transform.
        %/
        function rv = getTranslation(this) 
            rv = this.translation;
        end
        
        %*
        % Returns the X component of the transformation's translation.
        %
        % @rv = The x component of the transformation's translation.
        %/
        function rv = getX(this) 
            rv = this.translation.getX();
        end
        
        %*
        % Returns the Y component of the transformation's translation.
        %
        % @rv = The y component of the transformation's translation.
        %/
        function rv = getY(this) 
            rv = this.translation.getY();
        end
        
        %*
        % Returns the rotational component of the transformation.
        %
        % @rv = Reference to the rotational component of the transform.
        %/
        function rv = getRotation(this) 
            rv = this.rotation;
        end
        
        %*
        % Invert the transformation. This is useful for undoing a transformation.
        %
        % @rv = The inverted transformation.
        %/
        function rv = inverse(this) 
            % We are rotating the difference between the translations
            % using a clockwise rotation matrix. This transforms the global
            % delta into a local delta (relative to the initial pose).
            rv = Transform2d(...
            getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),...
            getRotation().unaryMinus());
        end
        
        function rv = toString(this) 
            rv = sprintf("Transform2d(%s, %s)", this.translation, this.rotation);
        end
        
        %*
        % Checks equality between this Transform2d and another object.
        %
        % @param obj The other object.
        % @rv = Whether the two objects are equal or not.
        %/
        function rv = equals(this, other) 
            rv = false;
            if isa(other, 'Transform2d') 
                rv = (other.translation.equals(this.translation)) ...
                    && (other.rotation.equals(this.rotation));
            end
        end
        
    end
end
