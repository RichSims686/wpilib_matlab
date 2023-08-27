classdef ChassisSpeeds < handle

    %*
     % Represents the speed of a robot chassis. Although this struct contains similar members compared
     % to a Twist2d, they do NOT represent the same thing. Whereas a Twist2d represents a change in pose
     % w.r.t to the robot frame of reference, this ChassisSpeeds struct represents a velocity w.r.t to
     % the robot frame of reference.
     %
     % <p>A strictly non-holonomic drivetrain, such as a differential drive, should never have a dy
     % component because it can never move sideways. Holonomic drivetrains such as swerve and mecanum
     % will often have all three components.
     %/

     properties
        %* Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) %/
        vxMetersPerSecond = 0;
        
        %* Represents sideways velocity w.r.t the robot frame of reference. (Left is +) %/
        vyMetersPerSecond = 0;
        
        %* Represents the angular velocity of the robot frame. (CCW is +) %/
        omegaRadiansPerSecond = 0;
     end
    
     methods
          %* Constructs a ChassisSpeeds with zeros for dx, dy, and theta. %/
        
          %*
           % Constructs a ChassisSpeeds object.
           %
           % @param vxMetersPerSecond Forward velocity.
           % @param vyMetersPerSecond Sideways velocity.
           % @param omegaRadiansPerSecond Angular velocity.
           %/
           function this = ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond) 
               if nargin == 3
                    this.vxMetersPerSecond = vxMetersPerSecond;
                    this.vyMetersPerSecond = vyMetersPerSecond;
                    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
               end
           end
            
           function rv = toString(this) 
                rv = sprintf("ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)", ...
                this.vxMetersPerSecond, this.vyMetersPerSecond, this.omegaRadiansPerSecond);
           end
     end


     methods (Static)
            %*
            % Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
            % object.
            %
            % @param vxMetersPerSecond The component of speed in the x direction relative to the field.
            %     Positive x is away from your alliance wall.
            % @param vyMetersPerSecond The component of speed in the y direction relative to the field.
            %     Positive y is to your left when standing behind the alliance wall.
            % @param omegaRadiansPerSecond The angular rate of the robot.
            % @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
            %     considered to be zero when it is facing directly away from your alliance station wall.
            %     Remember that this should be CCW positive.
            % @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
            %/

            %*
            % Converts a user provided field-relative ChassisSpeeds object into a robot-relative
            % ChassisSpeeds object.
            %
            % @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds in the field frame
            %     of reference. Positive x is away from your alliance wall. Positive y is to your left when
            %     standing behind the alliance wall.
            % @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
            %     considered to be zero when it is facing directly away from your alliance station wall.
            %     Remember that this should be CCW positive.
            % @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
            %/            
            function obj = fromFieldRelativeSpeeds(varargin)
                if nargin == 2
                    fieldRelativeSpeeds = varargin{1};
                    vxMetersPerSecond = fieldRelativeSpeeds.vxMetersPerSecond;
                    vyMetersPerSecond = fieldRelativeSpeeds.vyMetersPerSecond;
                    omegaRadiansPerSecond = fieldRelativeSpeeds.omegaRadiansPerSecond;
                    robotAngle = varargin{2};
                elseif nargin == 4
                    vxMetersPerSecond = varargin{1};
                    vyMetersPerSecond = varargin{2};
                    omegaRadiansPerSecond = varargin{3};
                    robotAngle = varargin{4};
                end
                obj = ChassisSpeeds(...
                    vxMetersPerSecond * robotAngle.getCos() + vyMetersPerSecond * robotAngle.getSin(), ...
                    -vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(), ...
                    omegaRadiansPerSecond);
            end

     end
end

