clear all;
close all;
clc;

HUMAN_ACTION_RANGE = 4.0;
RELATIVE_SPEED_MAX = 2.0 * 1.54;

v_dists = linspace(0.0, 1.5 * HUMAN_ACTION_RANGE);
for i=1:length(v_dists)
    SPEED_AGENT = 0.5;
    SPEED_OBSTACLE = 0.5;
    dist = v_dists(i);
    bs_var_dist_lin(i) = computeBehaviourStrengthLin(HUMAN_ACTION_RANGE, RELATIVE_SPEED_MAX, dist, SPEED_AGENT, SPEED_OBSTACLE);
    bs_var_dist_exp(i) = computeBehaviourStrengthExp(HUMAN_ACTION_RANGE, RELATIVE_SPEED_MAX, dist, SPEED_AGENT, SPEED_OBSTACLE);
end

figure;
plot(v_dists, bs_var_dist_lin)
hold on;
plot(v_dists, bs_var_dist_exp)
title("Behaviour strength as a function of distances between agents");
legend("linear", "exponential")


v_speed_agent = linspace(0.0, RELATIVE_SPEED_MAX);
for i=1:length(v_speed_agent)
    speed_agent = v_speed_agent(i);
    SPEED_OBSTACLE = 0.5;
    DIST = HUMAN_ACTION_RANGE / 2;
    bs_var_speed_agent_lin(i) = computeBehaviourStrengthLin(HUMAN_ACTION_RANGE, RELATIVE_SPEED_MAX, DIST, speed_agent, SPEED_OBSTACLE);
    bs_var_speed_agent_exp(i) = computeBehaviourStrengthExp(HUMAN_ACTION_RANGE, RELATIVE_SPEED_MAX, DIST, speed_agent, SPEED_OBSTACLE);
end

figure;
plot(v_speed_agent, bs_var_speed_agent_lin)
hold on;
plot(v_speed_agent, bs_var_speed_agent_exp)
title("Behaviour strength as a function of agent's speed");
legend("linear", "exponential")


v_speed_obstacle = linspace(0.0, RELATIVE_SPEED_MAX);
for i=1:length(v_speed_obstacle)
    SPEED_AGENT = 0.5;
    speed_obstacle = v_speed_obstacle(i);
    DIST = HUMAN_ACTION_RANGE / 2;
    bs_var_speed_obstacle_lin(i) = computeBehaviourStrengthLin(HUMAN_ACTION_RANGE, RELATIVE_SPEED_MAX, DIST, SPEED_AGENT, speed_obstacle);
    bs_var_speed_obstacle_exp(i) = computeBehaviourStrengthExp(HUMAN_ACTION_RANGE, RELATIVE_SPEED_MAX, DIST, SPEED_AGENT, speed_obstacle);
end

figure;
plot(v_speed_obstacle, bs_var_speed_obstacle_lin)
hold on;
plot(v_speed_obstacle, bs_var_speed_obstacle_exp)
title("Behaviour strength as a function of obstacle's speed");
legend("linear", "exponential")


% implementation of SocialConductor::computeBehaviourStrength
function val = computeBehaviourStrengthLin(human_action_range, relative_speed_max, dist_to_agent, speed_agent, speed_obstacle)
    % check if obstacle is too far away
	if (dist_to_agent > human_action_range)
		val = 0.0;
        return;
    end

	% in fact (SOCIAL_BEHAVIOUR_RANGE_END - SOCIAL_BEHAVIOUR_RANGE_START) but the start is 0.0
	a_dist = -1.0 / human_action_range;
	% form of a line equation for readability, the independent variable is `dist_to_agent`
	dist_factor = a_dist * dist_to_agent + 1.0;

	% compute impact of speed
	speed_arg = speed_agent + speed_obstacle;
	% trim
	if (speed_arg > relative_speed_max)
		speed_arg = relative_speed_max;
    end

	a_speed = 1.0 / relative_speed_max;
	speed_factor = a_speed * speed_arg;
    val = dist_factor * speed_factor;
end

function val = computeBehaviourStrengthExp(human_action_range, relative_speed_max, dist_to_agent, speed_agent, speed_obstacle)
    % check if obstacle is too far away
	if (dist_to_agent > human_action_range)
		val = 0.0;
        return;
    end
    
    mutual_speed = speed_agent + speed_obstacle;
    speed_factor = exp(mutual_speed) - 1.0;
    dist_factor = exp(-dist_to_agent);
    val = speed_factor * dist_factor;
end
