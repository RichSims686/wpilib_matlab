classdef LinearSystemSim < handle
    properties
        plant;
        x;
        y;
        u;
        measurementStdDevs;
    end

    methods
        function this = LinearSystemSim(system, measurementStdDevs)
            this.plant = system;

            this.x = zeros(size(system.getA(),1),1);
            this.y = zeros(size(system.getB(),2),1);
            this.u = zeros(size(system.getC(),1),1);
            this.measurementStdDevs = zeros(size(this.y));
            if nargin > 1
                this.measurementStdDevs = measurementStdDevs;
            end
        end

        function update(this, dtSeconds)
            this.x = this.updateX(this.x, this.u, dtSeconds);
            this.y = this.plant.calculateY(this.x, this.u);
            % add measurement noise
            this.y = this.y + StateSpaceUtil.makeWhiteNoiseVector(this.measurementStdDevs);
        end

        function rv = getOutput(this)
            rv = this.y;
        end

        function setInput(this, u)
            this.u = LinearSystemSim.clampInput(u);
        end

        function setState(this, state)
            this.x = state;
        end

        function rv = getCurrentDrawAmps(this) %#ok<MANU> 
            rv = 0;
        end

        function rv = updateX(this, currentXhat, u, dtSeconds)
            rv = this.plant.calculateX(currentXhat, u, dtSeconds);
        end

    end

    methods (Static)
        function rv = clampInput(u)
            rv = StateSpaceUtil.desaturateInputVector(u, RobotController.getBatteryVoltage());
        end


    end
end
