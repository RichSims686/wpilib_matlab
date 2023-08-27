classdef Units
    properties (Constant)
        kInchesPerFoot = 12.0;
        kMetersPerInch = 0.0254;
        kSecondsPerMinute = 60;
        kMillisecondsPerSecond = 1000;
        kKilogramsPerLb = 0.453592;
    end

    methods (Static)
        function rv = metersToFeet(meters) 
            rv = Units.metersToInches(meters) / Units.kInchesPerFoot;
        end

        function rv = feetToMeters(feet) 
            rv = Units.inchesToMeters(feet * Units.kInchesPerFoot);
        end

        function rv = metersToInches(meters) 
            rv = meters / Units.kMetersPerInch;
        end

        function rv = inchesToMeters(inches) 
            rv = inches * Units.kMetersPerInch;
        end

        function rv = degreesToRadians(degrees) 
            rv = deg2rad(degrees);
        end

        function rv =radiansToDegrees(radians) 
            rv = rad2deg(radians);
        end

        function rv = radiansToRotations(radians)
            rv = radians / (2*pi);
        end

        function rv = degreesToRotations(degrees) 
            rv = degrees / 360;
        end

        function rv = rotationsToDegrees(rotations) 
            rv = rotations * 360;
        end

        function rv = rotationsToRadians(rotations)
            rv  = rotations * 2*pi;
        end

        function rv = rotationsPerMinuteToRadiansPerSecond(rpm)
            rv = rpm * pi / (Units.kSecondsPerMinute / 2);
        end

        function rv = radiansPerSecondToRotationsPerMinute(radiansPerSecond)
            rv = radiansPerSecond * (Units.kSecondsPerMinute / 2) / pi;
        end
       
        function rv = millisecondsToSeconds(milliseconds) 
            rv = milliseconds / Units.kMillisecondsPerSecond;
        end

        function rv = secondsToMilliseconds(seconds)
            rv = seconds * Units.kMillisecondsPerSecond;
        end

        function rv = kilogramsToLbs(kilograms)
            rv = kilograms / Units.kKilogramsPerLb;
        end
  
        function rv = lbsToKilograms(lbs)
            rv = lbs * Units.kKilogramsPerLb;
        end
    end
end