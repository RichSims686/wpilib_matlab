classdef FlywheelSim < LinearSystemSim
    properties
        gearbox;
        gearing;
    end

    methods
        
        function this = FlywheelSim(gearbox, gearing, jKgMetersSquared)
            arguments
                gearbox (1,1) DCMotor
                gearing (1,1) double
                jKgMetersSquared (1,1)    double
            end

            plant = LinearSystemId.createFlywheelSystem(gearbox, jKgMetersSquared, gearing);

            this = this@LinearSystemSim(plant)
            this.gearbox = gearbox;
            this.gearing = gearing;
        end

        function rv = getAngularVelocityRadPerSec(this)
            rv = this.getOutput;
        end
   
        function rv = getAngularVelocityRPM(this)
            rv = Units.radiansPerSecondToRotationsPerMinute(this.getOutput);
        end

        function rv = getCurrentDrawAmps(this)
            rv = this.gearbox.getCurrent(this.getAngularVelocityRadPerSec() * this.gearing, this.u(1) * sign(this.u(1)));
        end

        function setInputVoltage(this, volts) 
            this.setInput(volts);
        end
    end
    
end
