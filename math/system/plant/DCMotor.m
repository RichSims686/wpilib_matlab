classdef DCMotor < handle
    properties
          nominalVoltageVolts;
          stallTorqueNewtonMeters;
          stallCurrentAmps;
          freeCurrentAmps;
          freeSpeedRadPerSec;
          rOhms;
          KvRadPerSecPerVolt;
          KtNMPerAmp;
    end

    methods
        function this = DCMotor(nominalVoltageVolts, ...
                stallTorqueNewtonMeters, ...
                stallCurrentAmps, ...
                freeCurrentAmps, ...
                freeSpeedRadPerSec, ...
                numMotors)
            this.nominalVoltageVolts = nominalVoltageVolts;
            this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors;
            this.stallCurrentAmps = stallCurrentAmps * numMotors;
            this.freeCurrentAmps = freeCurrentAmps * numMotors;
            this.freeSpeedRadPerSec = freeSpeedRadPerSec;
        
            this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
            this.KvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - this.rOhms * this.freeCurrentAmps);
            this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
        end

        function rv = getCurrent(obj, speedRadiansPerSec, voltageInputVolts)
            rv = -1.0 / obj.KvRadPerSecPerVolt / obj.rOhms * speedRadiansPerSec + 1.0 / obj.rOhms * voltageInputVolts;
        end

        function rv = getTorque(obj, currentAmpere) 
            rv = currentAmpere * obj.KtNMPerAmp;
        end

        function rv = getVoltage(obj, torqueNm, speedRadiansPerSec) 
            rv = 1.0 / obj.KvRadPerSecPerVolt * speedRadiansPerSec + 1.0 / obj.KtNMPerAmp * obj.rOhms * torqueNm;
        end

        function rv = getSpeed(obj, torqueNm, voltageInputVolts)
            rv = voltageInputVolts * obj.KvRadPerSecPerVolt - 1.0 / obj.KtNMPerAmp * torqueNm * obj.rOhms * obj.KvRadPerSecPerVolt;
        end

    end

    methods (Static)
        function rv = withReduction(obj, gearboxReduction) 
            rv = DCMotor(obj.nominalVoltageVolts, ...
                obj.stallTorqueNewtonMeters * gearboxReduction, ... 
                obj.stallCurrentAmps, ...
                obj.freeCurrentAmps, ...
                obj.freeSpeedRadPerSec / gearboxReduction, ...
                1);
        end

        function rv = getCIM(numMotors) 
            rv = DCMotor(12, 2.42, 133, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(5310), numMotors);
        end

        function rv = getVex775Pro(numMotors) 
            rv = DCMotor(12, 0.71, 134, 0.7, Units.rotationsPerMinuteToRadiansPerSecond(18730), numMotors);
        end

        function rv = getNEO(numMotors) 
            rv = DCMotor(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676), numMotors);
        end

        function rv = getMiniCIM(numMotors) 
            rv = DCMotor(12, 1.41, 89, 3, Units.rotationsPerMinuteToRadiansPerSecond(5840), numMotors);
        end

        function rv = getBag(numMotors) 
            rv = DCMotor(12, 0.43, 53, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(13180), numMotors);
        end

        function rv = getNeo550(numMotors) 
            rv = DCMotor(12, 0.97, 100, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(11000.0), numMotors);
        end

        function rv = getFalcon500(numMotors) 
            rv = DCMotor(12, 4.69, 257, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6380.0), numMotors);
        end    
    end
end
