function [vel_out] = findTwistWithAccLimits(vel_in, limits, minimum)
    %fprintf("[findTwistWithAccLimits] vel_in  %6.3f, %6.3f, %6.3f\n", vel_in(1), vel_in(2), vel_in(3));
    
    % decode limits
    acc_lim_x = limits(1);
    acc_lim_y = limits(2);
    acc_lim_th = limits(3);
    vel_min_x = limits(4);
    vel_min_y = limits(5);
    vel_min_th = limits(6);
    vel_max_x = limits(7);
    vel_max_y = limits(8);
    vel_max_th = limits(9);
    sim_granularity = limits(10);
    
    % partially similar to adjustTwistWithAccLimits free function
    vel_min_x_acc = max([vel_min_x, vel_in(1) - acc_lim_x * sim_granularity]);
    vel_min_y_acc = max([vel_min_y, vel_in(2) - acc_lim_y * sim_granularity]);
    vel_min_th_acc = max([vel_min_th, vel_in(3) - acc_lim_th * sim_granularity]);
    
    %fprintf("[findTwistWithAccLimits] vel_min_x_acc %6.3f, vel_min_y_acc %6.3f, vel_min_th_acc %6.3f\n", vel_min_x_acc, vel_min_y_acc, vel_min_th_acc);

    vel_max_x_acc = min([vel_max_x, vel_in(1) + acc_lim_x * sim_granularity]);
    vel_max_y_acc = min([vel_max_y, vel_in(2) + acc_lim_y * sim_granularity]);
    vel_max_th_acc = min([vel_max_th, vel_in(3) + acc_lim_th * sim_granularity]);
    %fprintf("[findTwistWithAccLimits] vel_max_x_acc %6.3f, vel_max_y_acc %6.3f, vel_max_th_acc %6.3f\n", vel_max_x_acc, vel_max_y_acc, vel_max_th_acc);

    vel_out = [];
    if minimum
        vel_out(1) = max([vel_min_x_acc, vel_min_x]);
        vel_out(2) = max([vel_min_y_acc, vel_min_y]);
        vel_out(3) = max([vel_min_th_acc, vel_min_th]);
    else
        vel_out(1) = min([vel_max_x_acc, vel_max_x]);
        vel_out(2) = min([vel_max_y_acc, vel_max_y]);
        vel_out(3) = min([vel_max_th_acc, vel_max_th]);
    end
    %fprintf("[findTwistWithAccLimits] vel_out %6.3f, %6.3f, %6.3f\n", vel_out(1), vel_out(2), vel_out(2));    
end
