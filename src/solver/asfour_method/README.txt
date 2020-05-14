This readme is written by: Sankaranarayanan Natarajan (sankar.natarajan@dfki.de)

This solver is based on the paper: 
"Human-like Motion of a Humanoid Robot Arm Based on a Closed-Form Solution of the Inverse Kinematics Problem".
If you read the paper you will understand the calculation of thetas are dependent on theta offsets and link twists of the manipulator.
However this solver was written using different theta offsets and link twists. Incase if your robot has a different theta offsets
and link twists simply by changing the config won't work. Either try to use the below given offset and twist configs or one need
to modify the source code.

For the below configuration the solver works fine:
  theta_offsets: [-1.57079632679, 1.57079632679, 0.0, 0.0, 1.57079632679, -1.57079632679,  0.0, -1.57079632679]
  link_twists: [-1.57079632679, 1.57079632679, -1.57079632679, 1.57079632679, -1.57079632679,  -1.57079632679, 0.0, 1.57079632679]

