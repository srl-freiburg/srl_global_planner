function dq = unicycleKinematics( t, q, v ,omega )

dq = [ v*cos(q(3)); v*sin(q(3)); omega ];


