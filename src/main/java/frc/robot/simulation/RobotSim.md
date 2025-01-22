# Robot Pieces
| Piece | XYZ Location|
| ---- | ----|
| Robot Base + Outside Elevator + Ramp | 0, 0, 1.5" |
| Elevator Stage | 4", 0, 3.25" |
| Head Carriage | 4", 0, 4.5" |
| Head | 10.5", 0, 16.625" |

# Poses
All component poses had to be seperate poses, array is not allowed.  Also, you add the DrivePose as a Robot names WcpCC25, then drag the other poses in this order as components inside that robot.
* Base: DrivePose
* Elevator: ElevatorPose
* Carriage: CarriagePose
* Head: ClawPose