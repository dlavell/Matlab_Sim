%--------------------------------------------------------------------------
% qrmodel.m
%
% Quadrotor dynamic model implemented in s-function
%
% David Cabecinhas
% Jun   2012
%--------------------------------------------------------------------------
function [sys,x0,str,ts] = mQX_model(t,x,u,flag)

% Parameters
ideal_motors = 1;
cd = 0.00;
m = 0.08093;
g = 9.81;
motor_pole = 1.5;


% Dispatch the flag.
switch flag,
    
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes(); % Initialization
        
    case 1
        sys = mdlDerivatives(t,x,u); % Calculate derivatives
        
    case 3
        sys = mdlOutputs(t,x,u); % Calculate outputs
        
    case { 2, 4, 9 } % Unused flags
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]); % Error handling
end
% End of qrmodel.

%==============================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the
% S-function.
%==============================================================
%
    function [sys,x0,str,ts] = mdlInitializeSizes()
        %
        % Call simsizes for a sizes structure, fill it in and convert it
        % to a sizes array.
        %
        sizes = simsizes;
        sizes.NumContStates  = 3+3+9+4;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 3+3+9+3;
        sizes.NumInputs      = 4;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        
        % Initialize the initial conditions.
%         x0(1:3) = [0.1266   -0.1772   -0.0312]';  %h0_minus_1
        x0(1:3) = [0.2096   -0.4043   -0.0296]';  %h0_1_good_for_t_10
        
        x0(4:6) = [0 0 0]';
        x0(7:15) = reshape([-0.9991   -0.0371   -0.0222
                            0.0370   -0.9993    0.0070
                           -0.0224    0.0062    0.9997],[9,1]); %h0_minus_1
%                         
        x0(7:15) = reshape([-0.9998   -0.0008   -0.0180
                            0.0005   -0.9999    0.0170
                           -0.0180    0.0170    0.9997],[9,1]); %h0_1_good_for_t_10
        x0(16:19) = [0.6 0 0 0]';
        
        % str is an empty matrix.
        str = [];
        % Initialize the array of sample times; in this example the sample
        % time is continuous, so set ts to 0 and its offset to 0.
        ts = [0 0];
    end % End of mdlInitializeSizes.

%==============================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%==============================================================
    function sys = mdlDerivatives(t,x,u)
        
        % p = x(1:3);
        v = x(4:6);
        R = reshape(x(7:15),3,3);
        u_real = x(16:19);
        
        u_real_dot = -(motor_pole)*2*pi*(u_real - u);
        
        % INSTANT motors
        if(ideal_motors)
            u_real = u;
        end
        
        % thrust = min(0.65- fb(3),1.0);
        % aileron = fb(2);
        % elevator = -fb(1);
        % yaw
        
        
        u_f = -9.81 * (113.3*u_real(1)+25.584) * 0.001;  % Thrust force (N)
        u_t(1,1) = 3.9062*u_real(2);
        u_t(2,1) = 3.9062*u_real(3);   % Pode nao estar muito certo
        u_t(3,1) = 3.7*u_real(4);
        
        % v_dot = params.M \ ( skew(params.M*v)*w + u_f);
        % w_dot = params.I \ ( skew(params.I*w)*w + skew(params.M*v)*v + u_t);
        sk_w = skew(u_t);
        
        v_dot = m * eye(3) \ ( - m * sk_w * v + [0 0 u_f]' + R'*[0 0 m*g]' - cd*v);
        p_dot = R*v;
        R_dot = R*sk_w;
        
        sys=[p_dot; v_dot; R_dot(:); u_real_dot];
    end % End of mdlDerivatives.
%


%==============================================================
% mdlOutputs
% Return the block outputs.
%==============================================================
%
    function sys = mdlOutputs(t,x,u)
        R = reshape(x(7:15),3,3);
                    
        if(ideal_motors)
            sys = [x(1:3); R*x(4:6); x(7:15); [0 0 0]'];
            % sys = [x(1:15); [0 0 0]'];
        else
            sys = [x(1:3); R*x(4:6); x(7:15); x(17:19)];
            % sys = x(1:19);
        end
    end % End of mdlOutputs.


end