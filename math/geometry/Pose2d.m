classdef Pose2d < handle

    %* Represents a 2D pose containing translational and rotational elements. */

    properties
        translation = Translation2d();
        rotation = Rotation2d();
    end

    methods

        %* Constructs a pose at the origin facing toward the positive X axis. */

        %*
        % Constructs a pose with the specified translation and rotation.
        %
        % @param translation The translational component of the pose.
        % @param rotation The rotational component of the pose.
        %/

        %*
        % Constructs a pose with x and y translations instead of a separate Translation2d.
        %
        % @param x The x component of the translational component of the pose.
        % @param y The y component of the translational component of the pose.
        % @param rotation The rotational component of the pose.
        %/

        function this = Pose2d(varargin)
            if nargin == 0
                this.translation = Translation2d();
                this.rotation = Rotation2d();
            elseif nargin == 2
                this.translation = varargin{1};
                this.rotation = varargin{2};
            elseif nargin == 3
                x = varargin{1};
                y = varargin{2};
                rotation = varargin{3};
                this.translation = Translation2d(x,y);
                this.rotation = rotation;
            end
        end
        
        %*
        % Transforms the pose by the given transformation and returns the transformed pose.
        %
        % <pre>
        % [x_new]    [cos, -sin, 0][transform.x]
        % [y_new] += [sin,  cos, 0][transform.y]
        % [t_new]    [  0,    0, 1][transform.t]
        % </pre>
        %
        % @param other The transform to transform the pose by.
        % @rv = The transformed pose.
        %/
        function rv = plus(this, other) 
            rv = this.transformBy(other);
        end
        
        %*
        % Returns the Transform2d that maps the one pose to another.
        %
        % @param other The initial pose of the transformation.
        % @rv = The transform that maps the other pose to the current pose.
        %/
        function rv = minus(this, other) 
            pose = this.relativeTo(other);
            rv = Transform2d(pose.getTranslation(), pose.getRotation());
        end
        
        %*
        % Returns the translation component of the transformation.
        %
        % @rv = The translational component of the pose.
        %/
        function rv = getTranslation(this) 
            rv = this.translation;
        end
        
        %*
        % Returns the X component of the pose's translation.
        %
        % @rv = The x component of the pose's translation.
        %/
        function rv = getX(this) 
            rv = this.translation.getX();
        end
        
        %*
        % Returns the Y component of the pose's translation.
        %
        % @rv = The y component of the pose's translation.
        %/
        function rv = getY(this) 
            rv = this.translation.getY();
        end
        
        %*
        % Returns the rotational component of the transformation.
        %
        % @rv = The rotational component of the pose.
        %/
        function rv = getRotation(this) 
            rv = this.rotation;
        end
        
        %*
        % Multiplies the current pose by a scalar.
        %
        % @param scalar The scalar.
        % @rv = The scaled Pose2d.
        %/
        function rv = times(this, scalar) 
            rv = Pose2d(this.translation.times(scalar), this.rotation.times(scalar));
        end
        
        %*
        % Divides the current pose by a scalar.
        %
        % @param scalar The scalar.
        % @rv = The scaled Pose2d.
        %/
        function rv = div(this, scalar) 
            rv = this.times(1.0 / scalar);
        end
        
        %*
        % Transforms the pose by the given transformation and returns the pose. See + operator for
        % the matrix multiplication performed.
        %
        % @param other The transform to transform the pose by.
        % @rv = The transformed pose.
        %/
        function rv = transformBy(this, other) 
            rv = Pose2d(...
                this.translation.plus(other.getTranslation().rotateBy(this.rotation)),...
                other.getRotation().plus(this.rotation));
        end
        
        %*
        % Returns the current pose relative to the given pose.
        %
        % <p>This function can often be used for trajectory tracking or pose stabilization algorithms to
        % get the error between the reference and the current pose.
        %
        % @param other The pose that is the origin of the coordinate frame that the current pose will
        %     be converted into.
        % @rv = The current pose relative to the origin pose.
        %/
        function rv = relativeTo(this, other) 
            transform = Transform2d(other, this);
            rv = Pose2d(transform.getTranslation(), transform.getRotation());
        end
        
        %*
        % Obtain a Pose2d from a (constant curvature) velocity.
        %
        % <p>See <a href="https:%file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
        % Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
        % derivation.
        %
        % <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
        % update. When the user runs exp() on the previous known field-relative pose with the argument
        % being the twist, the user will receive the field-relative pose.
        %
        % <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
        % pose forward in time.
        %
        % @param twist The change in pose in the robot's coordinate frame since the previous pose update.
        %     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
        %     degrees since the previous pose update, the twist would be Twist2d(0.01, 0.0,
        %     Units.degreesToRadians(0.5)).
        % @rv = The pose of the robot.
        %/
        function rv = exp(this, twist) 
            dx = twist.dx;
            dy = twist.dy;
            dtheta = twist.dtheta;
            
            sinTheta = sin(dtheta);
            cosTheta = cos(dtheta);
            
            if (abs(dtheta) < 1E-9) 
                s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
                c = 0.5 * dtheta;
            else 
                s = sinTheta / dtheta;
                c = (1 - cosTheta) / dtheta;
            end
            transform = Transform2d(...
                    Translation2d(dx * s - dy * c, dx * c + dy * s),...
                    Rotation2d(cosTheta, sinTheta));
            
            rv = this.plus(transform);
        end
        
        %*
        % Returns a Twist2d that maps this pose to the end pose. If c is the output of @code a.Log(b)end,
        % then @code a.Exp(c)end would yield b.
        %
        % @param endPose The end pose for the transformation.
        % @rv = The twist that maps this to end.
        %/
        function rv = log(this, endPose) 
            transform = endPose.relativeTo(this);
            dtheta = transform.getRotation().getRadians();
            halfDtheta = dtheta / 2.0;
            
            cosMinusOne = transform.getRotation().getCos() - 1;
            
            if (abs(cosMinusOne) < 1E-9) 
                halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
            else 
                halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
            end
            
            translationPart = transform.getTranslation().rotateBy(Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta)).times(hypot(halfThetaByTanOfHalfDtheta, halfDtheta));
            
            rv = Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
        end
        
        %*
        % Returns the nearest Pose2d from a list of poses. If two or more poses in the list have the same
        % distance from this pose, rv = the one with the closest rotation component.
        %
        % @param poses The list of poses to find the nearest.
        % @rv = The nearest Pose2d from the list.
        %/
        function rv = nearest(this, poses)
%             rv = Collections.min(
%                 poses,
%                 Comparator.comparing(
%                         (Pose2d other) -> this.getTranslation().getDistance(other.getTranslation()))
%                     .thenComparing(
%                         (Pose2d other) ->
%                             abs(this.getRotation().minus(other.getRotation()).getRadians())));
                        
            dx = [poses.translation.x] - this.translation.x;
            dy = [poses.translation.y] - this.translation.y;
            dist = hypot(dx, dy);
            minDist = min(dist);

            idx = find(dist == minDist);
            if numel(idx) == 1
                rv = poses(idx);
                return
            end

            error('TODO: nearest tiebreaker is closest rotation')

        end
        
        function rv = toString(this) 
            rv = sprintf("Pose2d(%s, %s)", this.translation, this.rotation);
        end
        
        %*
        % Checks equality between this Pose2d and another object.
        %
        % @param obj The other object.
        % @rv = Whether the two objects are equal or not.
        %/
        function rv = equals(this, other) 
            rv = false;
            if isa(other, 'Pose2d') 
              rv = other.this.translation.equals(this.translation) ...
                  && other.this.rotation.equals(this.rotation);
            end
        end
        
        function rv = interpolate(this, endValue, t) 
            if (t < 0) 
                rv = this;
                return
            elseif (t >= 1) 
                rv = endValue;
                return
            else 
                twist = this.log(endValue);
                scaledTwist = Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
                rv = this.exp(scaledTwist);
            end
        end
    end
end
    