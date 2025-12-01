% computeThetaAlphaBetaAngle2011

robot_vel1 = [0.0, 0.0, 0.0];
object_vel1 = [1.0, -1.0, 0.0];

cos_angle = dot(robot_vel1, object_vel1) / (norm(robot_vel1) * norm(object_vel1))
theta = acos(cos_angle)

robot_vel2 = [2.0, 3.0, 0.3];
object_vel2 = [1.0, -1.0, 0.0];
cos_angle = dot(robot_vel2, object_vel2) / (norm(robot_vel2) * norm(object_vel2))
theta = acos(cos_angle)

robot_vel3 = [1.0, -1.0, 0.0];
object_vel3 = [2.0, 3.0, 0.3];
cos_angle = dot(robot_vel3, object_vel3) / (norm(robot_vel3) * norm(object_vel3))
theta = acos(cos_angle)