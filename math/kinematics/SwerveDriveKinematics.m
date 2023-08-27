classdef SwerveDriveKinematics < handle

    %*
    % Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
    % module states (speed and angle).
    %
    % <p>The inverse kinematics (converting from a desired chassis velocity to individual module
    % states) uses the relative locations of the modules with respect to the center of rotation. The
    % center of rotation for inverse kinematics is also variable. This means that you can set your
    % center of rotation in a corner of the robot to perform special evasion maneuvers.
    %
    % <p>Forward kinematics (converting an array of module states into the overall chassis motion) is
    % performs the exact opposite of what inverse kinematics does. Since this is an overdetermined
    % system (more equations than variables), we use a least-squares approximation.
    %
    % <p>The inverse kinematics: [moduleStates] = [moduleLocations]% [chassisSpeeds] We take the
    % Moore-Penrose pseudoinverse of [moduleLocations] and then multiply by [moduleStates] to get our
    % chassis speeds.
    %
    % <p>Forward kinematics is also used for odometry -- determining the position of the robot on the
    % field using encoders and a gyro.
    %/

    properties
        inverseKinematics;
        forwardKinematics;
        
        numModules;
        modules;
        moduleStates = SwerveModuleState();
        prevCoR = Translation2d();
    end

    methods
        %*
        % Constructs a swerve drive kinematics object. This takes in a variable number of wheel locations
        % as Translation2d objects. The order in which you pass in the wheel locations is the same order
        % that you will receive the module states when performing inverse kinematics. It is also expected
        % that you pass in the module states in the same order when calling the forward kinematics
        % methods.
        %
        % @param wheelsMeters The locations of the wheels relative to the physical center of the robot.
        %/
        function this = SwerveDriveKinematics(wheelsMeters) 
            if (numel(wheelsMeters) < 2) 
                throw(MException('IllegalArgumentException',"A swerve drive requires at least two modules"));
            end
            this.numModules = numel(wheelsMeters);
            this.modules = wheelsMeters;
            this.moduleStates(this.numModules) = SwerveModuleState();
            this.inverseKinematics = zeros(this.numModules * 2, 3);
            
            for i=1:this.numModules
                this.inverseKinematics(2*i-1, :) = [1, 0, -this.modules(i).getY()];
                this.inverseKinematics(2*i-0, :) = [0, 1, +this.modules(i).getX()];
            end
            this.forwardKinematics = pinv(this.inverseKinematics);
        end
        
        %*
        % Performs inverse kinematics to return the module states from a desired chassis velocity. This
        % method is often used to convert joystick values into module speeds and angles.
        %
        % <p>This function also supports variable centers of rotation. During normal operations, the
        % center of rotation is usually the same as the physical center of the robot; therefore, the
        % argument is defaulted to that use case. However, if you wish to change the center of rotation
        % for evasive maneuvers, vision alignment, or for any other use case, you can do so.
        %
        % <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary),
        % the previously calculated module angle will be maintained.
        %
        % @param chassisSpeeds The desired chassis speed.
        % @param centerOfRotationMeters The center of rotation. For example, if you set the center of
        %     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
        %     component, the robot will rotate around that corner.
        % @return An array containing the module states. Use caution because these module states are not
        %     normalized. Sometimes, a user input may cause one of the module speeds to go above the
        %     attainable max velocity. Use the @link #desaturateWheelSpeeds(SwerveModuleState[], double)
        %     DesaturateWheelSpeedsend function to rectify this issue.
        %/
        function rv = toSwerveModuleStates(this, chassisSpeeds, centerOfRotationMeters) 
            if nargin == 2
                centerOfRotationMeters = Translation2d();
            end

            if (chassisSpeeds.vxMetersPerSecond == 0.0...
                && chassisSpeeds.vyMetersPerSecond == 0.0...
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) 

                newStates(this.numModules) = SwerveModuleState();
                for i=1:this.numModules
                    newStates(i) = SwerveModuleState(0.0, this.moduleStates(i).angle);
                end
            
                this.moduleStates = newStates;
                rv = this.moduleStates;
                return;
            end
            
            if (~centerOfRotationMeters.equals(this.prevCoR)) 
                for i=1:this.numModules
                    this.inverseKinematics(2*i-1, :) = [1, 0, -this.modules(i).getY() + centerOfRotationMeters.getY()];
                    this.inverseKinematics(2*i-0, :) = [0, 1, +this.modules(i).getX() - centerOfRotationMeters.getX()];
                end
                this.prevCoR = centerOfRotationMeters;
            end
            
            chassisSpeedsVector = zeros(3, 1);
            chassisSpeedsVector(:,1) = [...
                chassisSpeeds.vxMetersPerSecond,...
                chassisSpeeds.vyMetersPerSecond,...
                chassisSpeeds.omegaRadiansPerSecond];
            
            moduleStatesMatrix = this.inverseKinematics * chassisSpeedsVector;
            
            this.moduleStates(this.numModules) = SwerveModuleState();
            for i=1:this.numModules
                x = moduleStatesMatrix(2*i-1, 1);
                y = moduleStatesMatrix(2*i-0, 1);
                
                speed = hypot(x, y);
                angle = Rotation2d(x, y);
                
                this.moduleStates(i) = SwerveModuleState(speed, angle);
            end
            
            rv = this.moduleStates;
        end
        
       
        %*
        % Performs forward kinematics to return the resulting chassis state from the given module states.
        % This method is often used for odometry -- determining the robot's position on the field using
        % data from the real-world speed and angle of each module on the robot.
        %
        % @param wheelStates The state of the modules (as a SwerveModuleState type) as measured from
        %     respective encoders and gyros. The order of the swerve module states should be same as
        %     passed into the constructor of this class.
        % @return The resulting chassis speed.
        %/
        function rv = toChassisSpeeds(this, wheelStates) 
            if (numel(wheelStates) ~= this.numModules) 
                throw(MException('IllegalArgumentException', ...
                    "Number of modules is not consistent with number of wheel locations provided in constructor"));
            end            
            moduleStatesMatrix = zeros(this.numModules * 2, 1);
            
            for i=1:this.numModules
                module = wheelStates(i);
                moduleStatesMatrix(2*i-1) = module.speedMetersPerSecond * module.angle.getCos();
                moduleStatesMatrix(2*i-0) = module.speedMetersPerSecond * module.angle.getSin();
            end
            
            chassisSpeedsVector = this.forwardKinematics * moduleStatesMatrix;
            rv = ChassisSpeeds(chassisSpeedsVector(1), chassisSpeedsVector(2), chassisSpeedsVector(3));
        end
        
        %*
        % Performs forward kinematics to return the resulting chassis state from the given module states.
        % This method is often used for odometry -- determining the robot's position on the field using
        % data from the real-world speed and angle of each module on the robot.
        %
        % @param wheelDeltas The latest change in position of the modules (as a SwerveModulePosition
        %     type) as measured from respective encoders and gyros. The order of the swerve module states
        %     should be same as passed into the constructor of this class.
        % @return The resulting Twist2d.
        %/
        function rv = toTwist2d(this, wheelDeltas) 
            if (numel(wheelDeltas) ~= this.numModules) 
                throw(MException('IllegalArgumentException', ...
                    "Number of modules is not consistent with number of wheel locations provided in constructor"));
            end            
            moduleDeltaMatrix = zeros(this.numModules * 2, 1);
            
            for i=1:this.numModules
                module = wheelDeltas(i);
                moduleDeltaMatrix(2*i-1) = module.distanceMeters * module.angle.getCos();
                moduleDeltaMatrix(2*i-0) = module.distanceMeters * module.angle.getSin();
            end
            
            chassisDeltaVector = this.forwardKinematics * moduleDeltaMatrix;
            rv = Twist2d(chassisDeltaVector(1), chassisDeltaVector(2), chassisDeltaVector(3));
        end
    end

    methods (Static)
        

        
        function rv = desaturateWheelSpeeds(varargin) 
            if nargin == 2
                %*
                % Renormalizes the wheel speeds if any individual speed is above the specified maximum.
                %
                % <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
                % above the max attainable speed for the driving motor on that module. To fix this issue, one can
                % reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
                % absolute threshold, while maintaining the ratio of speeds between modules.
                %
                % @param moduleStates Reference to array of module states. The array will be mutated with the
                %     normalized speeds!
                % @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
                %/
                
                moduleStates = varargin{1};
                attainableMaxSpeedMetersPerSecond = varargin{2};

                realMaxSpeed = max([moduleStates.speedMetersPerSecond]);
                if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) 
                    for moduleState = moduleStates
                        moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
                    end
                end

            else
                %*
                % Renormalizes the wheel speeds if any individual speed is above the specified maximum, as well
                % as getting rid of joystick saturation at edges of joystick.
                %
                % <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
                % above the max attainable speed for the driving motor on that module. To fix this issue, one can
                % reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
                % absolute threshold, while maintaining the ratio of speeds between modules.
                %
                % @param moduleStates Reference to array of module states. The array will be mutated with the
                %     normalized speeds!
                % @param currentChassisSpeed The current speed of the robot
                % @param attainableMaxModuleSpeedMetersPerSecond The absolute max speed that a module can reach
                % @param attainableMaxTranslationalSpeedMetersPerSecond The absolute max speed that your robot
                %     can reach while translating
                % @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can
                %     reach while rotating
                %/
                
                moduleStates = varargin{1};
                currentChassisSpeed  = varargin{2};
                attainableMaxModuleSpeedMetersPerSecond = varargin{3};
                attainableMaxTranslationalSpeedMetersPerSecond = varargin{4};
                attainableMaxRotationalVelocityRadiansPerSecond = varargin{5};

                realMaxSpeed = max([moduleStates.speedMetersPerSecond]);
                
                if (attainableMaxTranslationalSpeedMetersPerSecond == 0 ...
                    || attainableMaxRotationalVelocityRadiansPerSecond == 0 ...
                    || realMaxSpeed == 0) 
                    return;
                end
                translationalK = hypot(currentChassisSpeed.vxMetersPerSecond, currentChassisSpeed.vyMetersPerSecond) / attainableMaxTranslationalSpeedMetersPerSecond;
                rotationalK = abs(currentChassisSpeed.omegaRadiansPerSecond) / attainableMaxRotationalVelocityRadiansPerSecond;
                k = max(translationalK, rotationalK);
                scale = min(k * attainableMaxModuleSpeedMetersPerSecond / realMaxSpeed, 1);
                for moduleState = moduleStates 
                    moduleState.speedMetersPerSecond = scale * moduleState.speedMetersPerSecond;
                end
            end
            rv = moduleStates;
        end
    end
end
