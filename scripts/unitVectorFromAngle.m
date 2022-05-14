ang = 0.785398 %-pi/3
x = [1; 0];
v = [cos(ang) -sin(ang); sin(ang) cos(ang)] * x
v_alpha_x = x(1) * cos(ang) - x(2) * sin(ang)
v_alpha_y = x(1) * sin(ang) + x(2) * cos(ang)
v == [v_alpha_x; v_alpha_y]