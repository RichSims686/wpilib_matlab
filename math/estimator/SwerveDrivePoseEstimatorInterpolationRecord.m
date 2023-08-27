classdef SwerveDrivePoseEstimatorInterpolationRecord < handle
    
    %*
    % Represents an odometry record. The record contains the inputs provided as well as the pose that
    % was observed based on these inputs, as well as the previous record and its inputs.
    %/

    properties
        % The pose observed given the current sensor inputs and the previous pose.
        poseMeters;
        
        % The current gyro angle.
        gyroAngle;
        
        % The distances and rotations measured at each module.
        modulePositions;
    end

    methods
        %*
        % Constructs an Interpolation Record with the specified parameters.
        %
        % @param pose The pose observed given the current sensor inputs and the previous pose.
        % @param gyro The current gyro angle.
        % @param wheelPositions The distances and rotations measured at each wheel.
        %/
        function this = SwerveDrivePoseEstimatorInterpolationRecord(poseMeters, gyro, modulePositions) 
            this.poseMeters = poseMeters;
            this.gyroAngle = gyro;
            this.modulePositions = modulePositions;
        end
        
        %*
        % rv = the interpolated record. This object is assumed to be the starting position, or lower
        % bound.
        %
        % @param endValue The upper bound, or end.
        % @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
        % @rv = The interpolated value.
        %/
        function rv = interpolate(this, endValue, t) 
            if (t < 0) 
                rv = this;
            elseif (t >= 1) 
                rv = endValue;
            else
                % Find the wheel distances.
                this.modulePositions(this.numModules) = SwerveModulePosition();
            
                % Find the distance travelled between this measurement and the interpolated measurement.
                moduleDeltas(this.numModules) = SwerveModulePosition();
            
                for i=1:this.numModules
                    ds = MathUtil.interpolate(...
                          this.modulePositions(i).distanceMeters,...
                          endValue.modulePositions(i).distanceMeters,...
                          t);

                    theta =  this.modulePositions(i).angle.interpolate(endValue.modulePositions(i).angle, t);

                    this.modulePositions(i) = SwerveModulePosition(ds, theta);
                    moduleDeltas(i) = SwerveModulePosition(ds - this.modulePositions(i).distanceMeters, theta);
                end
            
                % Find the gyro angle.
                gyro_lerp = this.gyroAngle.interpolate(endValue.gyroAngle, t);
            
                % Create a twist to represent this change based on the interpolated sensor inputs.
                twist = this.kinematics.toTwist2d(moduleDeltas);
                twist.dtheta = gyro_lerp.minus(this.gyroAngle).getRadians();
            
                rv = SwerveDrivePoseEstimatorInterpolationRecord(this.poseMeters.exp(twist), gyro_lerp, this.modulePositions);
            end
        end

        function rv = equals(this, other)
            if (this == other)
                rv = true;
                return;
            end
            if ~isa(other, 'SwerveDrivePoseEstimatorInterpolationRecord')
                rv = false;
                return;
            end

            rv = (this.gyroAngle == other.gyroAngle) ...
                  && (this.modulePositions == other.modulePositions) ...
                  && (this.poseMeters ==other.poseMeters);
        end       
    end
end
