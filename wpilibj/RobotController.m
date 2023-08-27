classdef RobotController
    methods (Static)
        %*
        % Read the microsecond timer from the FPGA.
        % 
        % Patched by AdvantageKit to read the syncronized timestamp. To access the real
        % FPGA time for performance analysis, call
        % {@code Logger.getInstance().getRealTimestamp()} instead.
        %
        % @return The current time in microseconds according to the FPGA.
        %/        
        function rv = getFPGATime()
            rv = floor(Timestamp.get() * 1e6);
        end

        %*
        % Read the battery voltage.
        %
        % @return The battery voltage in Volts.
        %/
        function rv = getBatteryVoltage()
            rv = 12.0;  % simulations
        end
    end
end