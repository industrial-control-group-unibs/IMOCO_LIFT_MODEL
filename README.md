# Lumped Parameter model of the Lift

Model: Lift without compensating rope

Assumption: the rope are consider viscous-elastic elements, using a lumped parameters model where the rope mass is located in the middle of the rope. The two half of the rope are modeled as spring-damper system.

The rope mass linearly depends on the rope length.

 

## System input
- _u_   motor torque
- _dc_  disturbance on the cabin side
- _dw_  disturbance on the counterweight side

## parameters
- _Mc_ mass of the cabin
- _Mw_ mass of the counterweight
- _Jp_ inertia of the pulley (and all the elements rigidly connected with it)
- _Jm_ inertia of the motor (and all the elements rigidly connected with it)
- _gearbox_  gearbox ratio (outer speed/inner speed)
- _LinearStiffness_ stiffness of 1 meter of rope
- _LinearDamping_  damping of 1 meter of rope
- _Rp_  radius of the pulley
- _mu_   linear density (mass of a 1 meter of rope)
- _BuildingHeight_ movement range of the lift
- _MotorViscousFriction_ motor friction
- _g_ gravitational acceleration
- _min_length_ minimum distance between the pulley and the cabin


## System state
- x1 : position of the counterweight
- x2 : velocity of the counterweight
- x3 : position of the rope mass (lumped in the middle of the robot) on the counterweight side
- x4 : velocity of the rope mass (lumped in the middle of the robot) on the counterweight side
- x5 : position of the rope mass (lumped in the middle of the robot) on the cabin side
- x6 : velocity of the rope mass (lumped in the middle of the robot) on the cabin side
- x7 : position of the cabin
- x8 : velocity of the cabin
- x9 : angular position of the motor
- x10: angular velocity of the motor

## Initial conditions
Assumption -> when x9=0 the cabin is in 0
