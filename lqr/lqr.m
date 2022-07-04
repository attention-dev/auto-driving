classdef lqr
    properties
        MaxNumIteration
        Tolerance
        Rho
        InitState
        PredictState
        OptimalCtrl
        CostState
        CostCtrl
    end
    
    methods
        function this = lqr(MaxNumIteration, Tolerance, Rho, InitState, varargin)
            this.MaxNumIteration = MaxNumIteration;
            this.Tolerance = Tolerance;
            this.Rho = Rho;
            this.InitState = InitState;
        end
        
        function [P_mat, K_mat, err, num_iteration] = solve(this, A, B, Q, Qf, R)
            P = Qf;
            num_iteration = 0;
            err = Inf;
            
            AT = A';
            BT = B';
            
            while (num_iteration < this.MaxNumIteration) && (err > this.Tolerance)
                P_next = Q + AT*P*A - (AT*P*B)*(R+BT*P*B)^-1*(BT*P*A);
                err = max(max(abs(P_next - P)));
                P = P_next;
                num_iteration = num_iteration + 1;
            end
            
            P_mat = P;
            K_mat = (R+BT*P*B)^-1 * BT*P*A;
        end
        
        function finite_horizon_data(~)
            
        end
        
    end
end
