classdef LinearSystem < handle
    properties
        A;
        B;
        C;
        D;
    end

    methods
        function this = LinearSystem(A,B,C,D)
            this.A  = A;
            this.B  = B;
            this.C  = C;
            this.D  = D;
        end

        function x = calculateX(this, x, clampedU, dtSeconds)
            [Ad, Bd] = Discretization.discretizeAB(this.A,this.B,dtSeconds);
            x = Ad*x + Bd*clampedU;
        end

        function y = calculateY(this, x, clampedU)
            y = this.C*x + this.D*clampedU;
        end

        function A = getA(this, row, col)
            if nargin > 1
                A = this.A(row+1,col+1);
            else
                A = this.A;
            end
        end

        function B = getB(this, row, col)
            if nargin > 1
                B = this.B(row+1,col+1);
            else
                B = this.B;
            end
        end

        function C = getC(this, row, col)
            if nargin > 1
                C = this.C(row+1,col+1);
            else
                C = this.C;
            end
        end

        function D = getD(this, row, col)
            if nargin > 1
                D = this.D(row+1,col+1);
            else
                D = this.D;
            end
        end

        
    end
end