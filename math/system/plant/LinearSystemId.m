classdef LinearSystemId
    methods (Static)

        %*
        % Create a state-space model of a flywheel system. The states of the system are [angular
        % velocity], inputs are [voltage], and outputs are [angular velocity].
        %
        % @param motor The motor (or gearbox) attached to the flywheel.
        % @param JKgMetersSquared The moment of inertia J of the flywheel.
        % @param G The reduction between motor and drum, as a ratio of output to input.
        % @return A LinearSystem representing the given characterized constants.
        % @throws IllegalArgumentException if JKgMetersSquared &lt;= 0 or G &lt;= 0.
        %/
        function obj = createFlywheelSystem(motor, JKgMetersSquared, G)
            if (JKgMetersSquared <= 0.0)
              throw(MException('IllegalArgumentException', "J must be greater than zero."));
            end
            if (G <= 0.0)
              throw(MException('IllegalArgumentException', "G must be greater than zero."));
            end
        
            A = -G * G * motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms * JKgMetersSquared);
            B = G * motor.KtNMPerAmp / (motor.rOhms * JKgMetersSquared);
            C = 1;
            D = 0;
        
            obj = LinearSystem(A,B,C,D);        
        end    


        %*
        % Create a state-space model of a DC motor system. The states of the system are [angular
        % position, angular velocity], inputs are [voltage], and outputs are [angular position, angular
        % velocity].
        %
        % @param motor The motor (or gearbox) attached to system.
        % @param JKgMetersSquared The moment of inertia J of the DC motor.
        % @param G The reduction between motor and drum, as a ratio of output to input.
        % @return A LinearSystem representing the given characterized constants.
        % @throws IllegalArgumentException if JKgMetersSquared &lt;= 0 or G &lt;= 0.
        %/    
        function obj = createDCMotorSystem(motor, JKgMetersSquared, G)
            if (JKgMetersSquared <= 0.0)
              throw(MException('IllegalArgumentException', "J must be greater than zero."));
            end
            if (G <= 0.0)
              throw(MException('IllegalArgumentException', "G must be greater than zero."));
            end
        
            A = [0, 1;
                 0, -G ...
                    * G ...
                    * motor.KtNMPerAmp ...
                    / (motor.KvRadPerSecPerVolt * motor.rOhms * JKgMetersSquared)];

            B = [0;
                 G * motor.KtNMPerAmp / (motor.rOhms * JKgMetersSquared)];
            C = eye(2);
            D = zeros(2);
        
            obj = LinearSystem(A,B,C,D);        
        end

        %*
        % Create a state-space model for a 1 DOF velocity system from its kV (volts/(unit/sec)) and kA
        % (volts/(unit/sec²). These constants cam be found using SysId. The states of the system are
        % [velocity], inputs are [voltage], and outputs are [velocity].
        %
        % <p>The distance unit you choose MUST be an SI unit (i.e. meters or radians). You can use the
        % @link edu.wpi.first.util.Unitsend class for converting between unit types.
        %
        % <p>The parameters provided by the user are from this feedforward model:
        %
        % <p>u = K_v v + K_a a
        %
        % @param kV The velocity gain, in volts/(unit/sec)
        % @param kA The acceleration gain, in volts/(unit/sec^2)
        % @return A LinearSystem representing the given characterized constants.
        % @throws IllegalArgumentException if kV &lt;= 0 or kA &lt;= 0.
        % @see <a href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
        %/
        function obj = identifyVelocitySystem(kV, kA)
            if (kV <= 0.0) 
                throw(MException('IllegalArgumentException',"Kv must be greater than zero."));
            end
            if (kA <= 0.0) 
                throw(MException('IllegalArgumentException',"Ka must be greater than zero."));
            end

            A = -kV / kA;
            B = 1.0 / kA;
            C = 1.0;
            D = 0.0;

            obj = LinearSystem(A,B,C,D);        
        end
    end
end