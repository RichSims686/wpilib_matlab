classdef Timer < handle
    
    %*
    % A timer class.
    %
    % <p>Note that if the user calls SimHooks.restartTiming(), they should also reset the timer so
    % get() won't rv = a negative duration.
    %/

    properties
        startTime;
        accumulatedTime;
        running;
    end


    methods (Static)

        %*
        % rv = the system clock time in seconds. rv = the time from the FPGA hardware clock in
        % seconds since the FPGA started.
        %
        % @rv = Robot running time in seconds.
        %/
        function rv = getFPGATimestamp() 
            rv = Timestamp.getFPGATimestamp();
        end

        function rv = getMsClock() 
            rv = Timestamp.get() * 1000.0;
        end


        %*
        % rv = the approximate match time. The FMS does not send an official match time to the robots,
        % but does send an approximate match time. The value will count down the time remaining in the
        % current period (auto or teleop). Warning: This is not an official time (so it cannot be used to
        % dispute ref calls or guarantee that a function will trigger before the match ends) The Practice
        % Match function of the DS approximates the behavior seen on the field.
        %
        % @rv = Time remaining in current match period (auto or teleop) in seconds
        %/
%         function rv = getMatchTime() 
%             rv = DriverStation.getMatchTime();
%         end
        
        %*
        % Pause the thread for a specified time. Pause the execution of the thread for a specified period
        % of time given in seconds. Motors will continue to run at their last assigned values, and
        % sensors will continue to update. Only the task containing the wait will pause until the wait
        % time is expired.
        %
        % @param seconds Length of time to pause
        %/
%         function  delay(seconds) 
%             try 
%                 Thread.sleep((long) (seconds * 1e3));
%             catch (final InterruptedException ex) 
%                 Thread.currentThread().interrupt();
%             end
%         end    
    end
    
    
    
    methods    
        
        %* Timer constructor. */
        function this = Timer() 
            this.reset();
        end

        
        %*
        % Get the current time from the timer. If the clock is running it is derived from the current
        % system clock the start time stored in the timer class. If the clock is not running, then rv =
        % the time when it was last stopped.
        %
        % @rv = Current time value for this timer in seconds
        %/
        function rv = get(this) 
            if (this.running) 
                rv = this.accumulatedTime + (Timer.getMsClock() - this.startTime) / 1000.0;
            else 
                rv = this.accumulatedTime;
            end
        end
        
        %*
        % Reset the timer by setting the time to 0.
        %
        % <p>Make the timer startTime the current time so new requests will be relative now.
        %/
        function reset(this) 
            this.accumulatedTime = 0;
            this.startTime = Timer.getMsClock();
        end
        
        %*
        % Start the timer running. Just set the running flag to true indicating that all time requests
        % should be relative to the system clock. Note that this method is a no-op if the timer is
        % already running.
        %/
        function start(this) 
            if (~this.running) 
                this.startTime = Timer.getMsClock();
                this.running = true;
            end
        end
        
        %*
        % Restart the timer by stopping the timer, if it is not already stopped, resetting the
        % accumulated time, then starting the timer again. If you want an event to periodically reoccur
        % at some time interval from the start time, consider using advanceIfElapsed() instead.
        %/
        function restart(this) 
            if (this.running) 
                this.stop();
            end
            this.reset();
            this.start();
        end
        
        %*
        % Stop the timer. This computes the time as of now and clears the running flag, causing all
        % subsequent time requests to be read from the accumulated time rather than looking at the system
        % clock.
        %/
        function stop(this) 
            this.accumulatedTime = this.get();
            this.running = false;
        end
        
        %*
        % Check if the period specified has passed.
        %
        % @param seconds The period to check.
        % @rv = Whether the period has passed.
        %/
        function rv = hasElapsed(this, seconds) 
            rv = this.get() >= seconds;
        end
        
        %*
        % Check if the period specified has passed and if it has, advance the start time by that period.
        % This is useful to decide if it's time to do periodic work without drifting later by the time it
        % took to get around to checking.
        %
        % @param seconds The period to check.
        % @rv = Whether the period has passed.
        %/
        function rv = advanceIfElapsed(this, seconds) 
            if (this.get() >= seconds) 
                % Advance the start time by the period.
                % Don't set it to the current time... we want to avoid drift.
                this.startTime = this.startTime + seconds * 1000;
                rv = true;
            else 
                rv = false;
            end
        end
    end
end
