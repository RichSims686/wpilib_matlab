classdef SwerveDriveOdometry < handle

        %*
        % Class for swerve drive odometry. Odometry allows you to track the robot's position on the field
        % over a course of a match using readings from your swerve drive encoders and swerve azimuth
        % encoders.
        %
        % <p>Teams can use odometry during the autonomous period for complex tasks like path following.
        % Furthermore, odometry can be used for latency compensation when using computer-vision systems.
        %/

    properties
        kinematics;
        poseMeters;
        
        gyroOffset;
        previousAngle;
        numModules;
        previousModulePositions = SwerveModulePosition.empty();
    end

    methods
        %*
        % Constructs a SwerveDriveOdometry object.
        %
        % @param kinematics The swerve drive kinematics for your drivetrain.
        % @param gyroAngle The angle reported by the gyroscope.
        % @param modulePositions The wheel positions reported by each module.
        % @param initialPose The starting position of the robot on the field.
        %/

        %*
        % Constructs a SwerveDriveOdometry object with the default pose at the origin.
        %
        % @param kinematics The swerve drive kinematics for your drivetrain.
        % @param gyroAngle The angle reported by the gyroscope.
        % @param modulePositions The wheel positions reported by each module.
        %/

        function this = SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPose) 
            if nargin == 3
                initialPose = Pose2d();
            end
            this.kinematics = kinematics;
            this.poseMeters = initialPose;
            this.gyroOffset = this.poseMeters.getRotation().minus(gyroAngle);
            this.previousAngle = initialPose.getRotation();
            this.numModules = numel(modulePositions);
            
            this.previousModulePositions(this.numModules) = SwerveModulePosition;
            for index = 1:this.numModules
                this.previousModulePositions(index) = SwerveModulePosition(...
                      modulePositions(index).distanceMeters, modulePositions(index).angle);
            end
        end
        

        %*
        % Resets the robot's position on the field.
        %
        % <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
        % automatically takes care of offsetting the gyro angle.
        %
        % <p>Similarly, module positions do not need to be reset in user code.
        %
        % @param gyroAngle The angle reported by the gyroscope.
        % @param modulePositions The wheel positions reported by each module.,
        % @param pose The position on the field that your robot is at.
        %/
        function resetPosition(this, gyroAngle, modulePositions, pose) 
            if (numel(modulePositions) ~= this.numModules) 
                throw(MException('IllegalArgumentException', ...
                    "Number of modules is not consistent with number of wheel locations provided in constructor"));
            end
            
            this.poseMeters = pose;
            this.previousAngle = pose.getRotation();
            this.gyroOffset = this.poseMeters.getRotation().minus(gyroAngle);
            for index=1:this.numModules
                this.previousModulePositions(index) = SwerveModulePosition(...
                      modulePositions(index).distanceMeters, modulePositions(index).angle);
            end
        end
        
        %*
        % Returns the position of the robot on the field.
        %
        % @rv = The pose of the robot (x and y are in meters).
        %/
        function rv = getPoseMeters(this) 
            rv = this.poseMeters;
        end
        
        %*
        % Updates the robot's position on the field using forward kinematics and integration of the pose
        % over time. This method automatically calculates the current time to calculate period
        % (difference between two timestamps). The period is used to calculate the change in distance
        % from a velocity. This also takes in an angle parameter which is used instead of the angular
        % rate that is calculated from forward kinematics.
        %
        % @param gyroAngle The angle reported by the gyroscope.
        % @param modulePositions The current position of all swerve modules. Please provide the positions
        %     in the same order in which you instantiated your SwerveDriveKinematics.
        % @rv = The pose of the robot.
        %/
        function rv = update(this, gyroAngle, modulePositions) 
            if (numel(modulePositions) ~= this.numModules) 
              throw(MException('IllegalArgumentException', ...
                  "Number of modules is not consistent with number of wheel locations provided in constructor"));
            end
            
            moduleDeltas(this.numModules) = SwerveModulePosition();
            for index=1:this.numModules
                current = modulePositions(index);
                previous = this.previousModulePositions(index);
                
                moduleDeltas(index) = SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
                previous.distanceMeters = current.distanceMeters;
            end
            
            angle = gyroAngle.plus(this.gyroOffset);
            
            twist = this.kinematics.toTwist2d(moduleDeltas);
            twist.dtheta = angle.minus(this.previousAngle).getRadians();
            
            newPose = this.poseMeters.exp(twist);
            
            this.previousAngle = angle;
            this.poseMeters = Pose2d(newPose.getTranslation(), angle);
            
            rv = this.poseMeters;
        end
    end
end
