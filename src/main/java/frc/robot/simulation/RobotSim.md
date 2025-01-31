# Poses
All component poses had to be seperate poses, array is not allowed.  Also, you add the DrivePose as a Robot named WcpCC25, then drag the other poses in this order as components inside that robot.
* Base: DrivePose
* Elevator: ElevatorPose
* Carriage: CarriagePose
* Head: ClawPose

Testing poses:
* ZeroPose - all components set to zero to allow initial placement of pieces.
* TestingPose - Pose where all the directions are set in NetworkTables to test placement in the robot.

# Commanding the Robot to Move
To make this model move, there are 2 NetworkTable values that can be set in dashboards or code to have the model move the poses for you:
* /Simulation/Claw Height - This sets how high the claw is in inches.  This is based on the pivot point of the claw.  That point seems to be 16.625" to the floor at "resting position".  The elevator seems to go between 16-80" with no problems, but the math seems to work with much bigger values.  There is no limit in simulation.
* /Simulation/Claw Angle - This sets the claw angle in degrees.  The claw simulation does not simulate collisions, but there is a hard stop at 0* (starting location tucked in), and it seems in CAD the head would start colliding at 180* around.  The hard stop would hit the bottom of the bar at ~280*.

# Rigging the robot
To setup the robot, AdvantageScope recommends setting all the poses to 0,0,0 with zero rotation.  Then, modify each mechanism to have the rotational origin to be at the zero point.  It will make the math a LOT easier to get it in the right position on the robot with the model and origin zeroed on the field.  Using the Axis view will help a lot here.

We publish a zero pose to help with rigging, and a TestPose for moving parts in testing.

Our model is somewhat rigged wrong, the elevator segment and head carriage are moved into the proper locations per the CAD model, not to the zero origin.  This did not affect their function, as they do not rotate.  For the claw head, since that does rotate, you must put the axle of the rotation on the zero point.  Then in code, we have a translation to get the head to the right spot, and 

This demo requires WpiLib 2025.2.1.  The function rotateAround was supposed to be in this release, but was missed (see https://github.com/wpilibsuite/allwpilib/issues/7657).  The Translation3d.rotateAround() did make it in, so we made a wrapper to support it for now.

Look at this documentation for more help, especially the video.  https://docs.advantagescope.org/more-features/custom-assets#3d-robot-models  Rigging the robot was not easy, and took some time to get setup.

# Robot Pieces
| Piece | XYZ Location|
| ---- | ----|
| Robot Base + Outside Elevator + Ramp | 0, 0, 1.5" |
| Elevator Stage | 4", 0, 3.25" |
| Head Carriage | 4", 0, 4.5" |
| Head | 10.5", 0, 16.625" |