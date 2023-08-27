classdef SwerveDrivePoseEstimator < handle

    %*%
    % This class wraps @link SwerveDriveOdometry Swerve Drive Odometryend to fuse latency-compensated
    % vision measurements with swerve drive encoder distance measurements. It is intended to be a
    % drop-in replacement for @link edu.wpi.first.math.kinematics.SwerveDriveOdometryend.
    %
    % <p>@link SwerveDrivePoseEstimator#updateend should be called every robot loop.
    %
    % <p>@link SwerveDrivePoseEstimator#addVisionMeasurementend can be called as infrequently as you
    % want; if you never call it, then this class will behave as regular encoder odometry.
    %/

    properties (Constant)
        kBufferDurationSeconds = 1.5;
    end

    properties
        kinematics;
        odometry;
        q = zeros(3,1);
        numModules;
        visionK = zeros(3,3);
                   
        poseBuffer = TimeInterpolatableBuffer.createBuffer(...
            @SwerveDrivePoseEstimatorInterpolationRecord.interpolate, ...
            SwerveDrivePoseEstimator.kBufferDurationSeconds);
    end

    methods 
        %*
        % Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and vision
        % measurements.
        %
        % <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
        % and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
        % meters for x, 0.9 meters for y, and 0.9 radians for heading.
        %
        % @param kinematics A correctly-configured kinematics object for your drivetrain.
        % @param gyroAngle The current gyro angle.
        % @param modulePositions The current distance measurements and rotations of the swerve modules.
        % @param initialPoseMeters The starting pose estimate.
        %/

        %*
        % Constructs a SwerveDrivePoseEstimator.
        %
        % @param kinematics A correctly-configured kinematics object for your drivetrain.
        % @param gyroAngle The current gyro angle.
        % @param modulePositions The current distance and rotation measurements of the swerve modules.
        % @param initialPoseMeters The starting pose estimate.
        % @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
        %     in meters, and heading in radians). Increase these numbers to trust your state estimate
        %     less.
        % @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
        %     in meters, y position in meters, and heading in radians). Increase these numbers to trust
        %     the vision pose measurement less.
        %/
        function this = SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionMeasurementStdDevs)
            if nargin == 4
                stateStdDevs = [0.1; 0.1; 0.1];
                visionMeasurementStdDevs = [0.9; 0.9; 0.9];
            end

            this.kinematics = kinematics;
            this.odometry = SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters);
            
            this.q = stateStdDevs .^ 2;
            
            this.numModules = numel(modulePositions);
            
            this.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        end
        
        
        %*
        % Sets the pose estimator's trust of global measurements. This might be used to change trust in
        % vision measurements after the autonomous period, or to change trust as distance to a vision
        % target increases.
        %
        % @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
        %     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
        %     theta]ᵀ, with units in meters and radians.
        %/
        function setVisionMeasurementStdDevs(this, visionMeasurementStdDevs) 
            r = visionMeasurementStdDevs .^ 2;
            
            % Solve for closed form Kalman gain for continuous Kalman filter with A = 0
            % and C = I. See wpimath/algorithms.md.
            for row = 1:3
                if (this.q(row) == 0.0) 
                    this.visionK(row, row) = 0.0;
                else 
                    this.visionK(row, row) = this.q(row, 1) / (this.q(row, 1) + sqrt(this.q(row, 1) * r(row)));
                end
            end
        end
        
        %*
        % Resets the robot's position on the field.
        %
        % <p>The gyroscope angle does not need to be reset in the user's robot code. The library
        % automatically takes care of offsetting the gyro angle.
        %
        % @param gyroAngle The angle reported by the gyroscope.
        % @param modulePositions The current distance measurements and rotations of the swerve modules.
        % @param poseMeters The position on the field that your robot is at.
        %/
        function resetPosition(this, gyroAngle, modulePositions, poseMeters) 
            % Reset state estimate and error covariance
            this.odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
            this.poseBuffer.clear();
        end
        
        %*
        % Gets the estimated robot pose.
        %
        % @rv = The estimated robot pose in meters.
        %/
        function rv = getEstimatedPosition(this) 
            rv = this.odometry.getPoseMeters();
        end
        
        %*
        % Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
        % while still accounting for measurement noise.
        %
        % <p>This method can be called as infrequently as you want, as long as you are calling @link
        % SwerveDrivePoseEstimator#updateend every loop.
        %
        % <p>To promote stability of the pose estimate and make it robust to bad vision data, we
        % recommend only adding vision measurements that are already within one meter or so of the
        % current pose estimate.
        %
        % @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
        % @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
        %     don't use your own time source by calling @link
        %     SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])end then you
        %     must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is
        %     the same epoch as @link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()end.) This means that
        %     you should use @link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()end as your time source
        %     or sync the epochs.
        %/

        %*
        % Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
        % while still accounting for measurement noise.
        %
        % <p>This method can be called as infrequently as you want, as long as you are calling @link
        % SwerveDrivePoseEstimator#updateend every loop.
        %
        % <p>To promote stability of the pose estimate and make it robust to bad vision data, we
        % recommend only adding vision measurements that are already within one meter or so of the
        % current pose estimate.
        %
        % <p>Note that the vision measurement standard deviations passed into this method will continue
        % to apply to future measurements until a subsequent call to @link
        % SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)end or this method.
        %
        % @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
        % @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
        %     don't use your own time source by calling @link
        %     SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])end, then
        %     you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
        %     timestamp is the same epoch as @link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()end).
        %     This means that you should use @link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()end as
        %     your time source in this case.
        % @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
        %     in meters, y position in meters, and heading in radians). Increase these numbers to trust
        %     the vision pose measurement less.
        %/
        
        function addVisionMeasurement(this, visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs) 
            if nargin == 4
                this.setVisionMeasurementStdDevs(visionMeasurementStdDevs);                
            end

            % Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
            try 
                if (this.poseBuffer.getInternalBufferLastKey() - this.kBufferDurationSeconds > timestampSeconds) 
                    return;
                end
            catch
                return;
            end
            
            % Step 1: Get the pose odometry measured at the moment the vision measurement was made.
            sample = this.poseBuffer.getSample(timestampSeconds);
            
            if (sample.isEmpty()) 
                return;
            end
            
            % Step 2: Measure the twist between the odometry pose and the vision pose.
            twist = sample.get().poseMeters.log(visionRobotPoseMeters);
            
            % Step 3: We should not trust the twist entirely, so instead we scale this twist by a Kalman
            % gain matrix representing how much we trust vision measurements compared to our current pose.
            k_times_twist = this.visionK * [twist.dx; twist.dy; twist.dtheta];
            
            % Step 4: Convert back to Twist2d.
            scaledTwist = Twist2d(k_times_twist(1), k_times_twist(2), k_times_twist(3));
            
            % Step 5: Reset Odometry to state at sample with vision adjustment.
            this.odometry.resetPosition(...
                sample.get().gyroAngle,...
                sample.get().modulePositions,...
                sample.get().poseMeters.exp(scaledTwist));
            
            % Step 6: Record the current pose to allow multiple measurements from the same timestamp
            this.poseBuffer.addSample(...
                timestampSeconds,...
                SwerveDrivePoseEstimatorInterpolationRecord(getEstimatedPosition(), sample.get().gyroAngle, sample.get().modulePositions));
            
            % Step 7: Replay odometry inputs between sample time and latest recorded sample to update the
            % pose buffer and correct odometry.
            map = this.poseBuffer.getInternalBufferTailMap(timestampSeconds);
            for key = keys(map)
                this.updateWithTime(key, map(key).gyroAngle, map(key).modulePositions);
            end
        end
        
        
        %*
        % Updates the pose estimator with wheel encoder and gyro information. This should be called every
        % loop.
        %
        % @param gyroAngle The current gyro angle.
        % @param modulePositions The current distance measurements and rotations of the swerve modules.
        % @rv = The estimated pose of the robot in meters.
        %/
        function rv = update(this, gyroAngle, modulePositions) 
            rv = this.updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, modulePositions);
        end
        
        %*
        % Updates the pose estimator with wheel encoder and gyro information. This should be called every
        % loop.
        %
        % @param currentTimeSeconds Time at which this method was called, in seconds.
        % @param gyroAngle The current gyroscope angle.
        % @param modulePositions The current distance measurements and rotations of the swerve modules.
        % @rv = The estimated pose of the robot in meters.
        %/
        function rv = updateWithTime(this, currentTimeSeconds, gyroAngle, modulePositions) 
            if (numel(modulePositions) ~= this.numModules) 
                throw(MException('IllegalArgumentException', ...
                  "Number of modules is not consistent with number of wheel locations provided in constructor"));
            end
            
            internalModulePositions(this.numModules) = SwerveModulePosition();
            for i=1:this.numModules
                internalModulePositions(i) = ...
                  SwerveModulePosition(modulePositions(i).distanceMeters, modulePositions(i).angle);
            end
            
            this.odometry.update(gyroAngle, internalModulePositions);
            
            this.poseBuffer.addSample(...
                currentTimeSeconds,...
                SwerveDrivePoseEstimatorInterpolationRecord(this.getEstimatedPosition(), gyroAngle, internalModulePositions));
            
            rv = this.getEstimatedPosition();
        end
       
    end
end
