classdef Twist2d < handle
    
    %*
    % A change in distance along a 2D arc since the last pose update. We can use ideas from
    % differential calculus to create new Pose2d objects from a Twist2d and vice versa.
    %
    % <p>A Twist can be used to represent a difference between two poses.
    %/
    properties
        %* Linear "dx" component. */
        dx = 0;
    
        %* Linear "dy" component. */
        dy = 0;
    
        %* Angular "dtheta" component (radians). */
        dtheta = 0;
    end

    methods

        %*
        % Constructs a Twist2d with the given values.
        %
        % @param dx Change in x direction relative to robot.
        % @param dy Change in y direction relative to robot.
        % @param dtheta Change in angle relative to robot.
        %/
        function this = Twist2d(dx, dy, dtheta) 
            if nargin == 0
                this.dx = 0;
                this.dy = 0;
                this.dtheta = 0;
            else
                this.dx = dx;
                this.dy = dy;
                this.dtheta = dtheta;
            end
        end
        
        function rv = toString(this) 
            rv = sprintf("Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", this.dx, this.dy, this.dtheta);
        end
        
        %*
        % Checks equality between this Twist2d and another object.
        %
        % @param obj The other object.
        % @rv = Whether the two objects are equal or not.
        %/
        function rv = equals(other) 
            rv = false;
            if isa(obj, 'Twist2d') 
              rv = abs(other.dx - this.dx) < 1E-9 ...
                  && abs(other.dy -this. dy) < 1E-9 ...
                  && abs(other.dtheta - this.dtheta) < 1E-9;
            end
        end
    end
        
end

