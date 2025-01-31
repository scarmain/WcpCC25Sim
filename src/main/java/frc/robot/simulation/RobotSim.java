package frc.robot.simulation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

//you cannot pass in an array of components, each component must be independent.
public class RobotSim implements Runnable {
    StructPublisher<Pose3d> drivetrainPub;
    StructPublisher<Pose3d> elevatorPub;
    StructPublisher<Pose3d> carriagePub;
    StructPublisher<Pose3d> clawPub;
    StructPublisher<Pose3d> zeroPub;
    StructPublisher<Pose3d> testPub;
    StructArrayPublisher<Pose3d> coralPub;
    Pose3d componentPoses[];
    DoubleEntry heightSub;
    DoubleEntry clawSub;
    DoubleEntry testX, testY, testZ, testRoll, testPitch, testYaw;
    Supplier<Pose2d> driveSupplier;
    //this is the height where the claw pivot sits at rest
    static final double INITIAL_HEIGHT = 16.625;

    public RobotSim(Supplier<Pose2d> robotPos) {
        driveSupplier = robotPos;
        var inst = NetworkTableInstance.getDefault();
        zeroPub = inst.getStructTopic("/Simulation/ZeroPose", Pose3d.struct).publish();
        zeroPub.set(new Pose3d());
        testPub = inst.getStructTopic("/Simulation/TestPose", Pose3d.struct).publish();
        drivetrainPub = inst.getStructTopic("/Simulation/DrivePose", Pose3d.struct).publish();
        elevatorPub = inst.getStructTopic("/Simulation/ElevatorPose", Pose3d.struct).publish();
        carriagePub = inst.getStructTopic("/Simulation/CarriagePose", Pose3d.struct).publish();
        clawPub = inst.getStructTopic("/Simulation/ClawPose", Pose3d.struct).publish();
        coralPub = inst.getStructArrayTopic("/Simulation/CoralPoses", Pose3d.struct).publish();

        //NT values to allow the model to be changed
        heightSub = inst.getDoubleTopic("/Simulation/Claw Height").getEntry(INITIAL_HEIGHT);
        heightSub.set(INITIAL_HEIGHT);
        clawSub = inst.getDoubleTopic("/Simulation/Claw Angle").getEntry(0);
        clawSub.set(0);

        testX = inst.getDoubleTopic("/Simulation/TestPose/X").getEntry(0);
        testY = inst.getDoubleTopic("/Simulation/TestPose/Y").getEntry(0);
        testZ = inst.getDoubleTopic("/Simulation/TestPose/Z").getEntry(0);
        testRoll = inst.getDoubleTopic("/Simulation/TestPose/Roll").getEntry(0);
        testPitch = inst.getDoubleTopic("/Simulation/TestPose/Pitch").getEntry(0);
        testYaw = inst.getDoubleTopic("/Simulation/TestPose/Yaw").getEntry(0);
        //have to initialize all values so they appear in NT
        testX.set(0);
        testY.set(0);
        testZ.set(0);
        testRoll.set(0);
        testPitch.set(0);
        testYaw.set(0);
        //set bumper distances
    }

    @Override
    public void run() {
        var totalHeight = heightSub.get() - INITIAL_HEIGHT;
        var drivePose = new Pose3d(driveSupplier.get());

        //test pose
        testPub.set(new Pose3d(new Translation3d(testX.get(), testY.get(), testZ.get()), 
            new Rotation3d(testRoll.get(), testPitch.get(), testYaw.get())));

        //set drivetrain
        drivetrainPub.set(drivePose);

        //set elevator
        elevatorPub.set(new Pose3d().transformBy(
            new Transform3d(0., 0., Units.inchesToMeters(totalHeight/2), 
            Rotation3d.kZero)));
        carriagePub.set(new Pose3d().transformBy(
            new Transform3d(0., 0., Units.inchesToMeters(totalHeight), 
            Rotation3d.kZero)));

        //initialize the claw pose
        //the claw has to be positioned so the shaft rotates around the axises, then transposed up to the right location
        var clawPose = new Pose3d();
        clawPose = clawPose.transformBy(
            new Transform3d(0.2667, 0, Units.inchesToMeters(heightSub.get()), 
            new Rotation3d(0, Math.toRadians(clawSub.get()), 0)));
        clawPub.set(clawPose);

        var coral = new Pose3d();
        //position this coral on the claw
        coral = coral.transformBy(
            new Transform3d(Units.inchesToMeters(5), 0, Units.inchesToMeters(11+heightSub.get()), 
            new Rotation3d(0, Math.toRadians(18), 0)));

        //then rotate it around the claw
        coral = rotateAround(coral, 
            new Translation3d(0.2667, 0, Units.inchesToMeters(heightSub.get())), 
            new Rotation3d(0, Math.toRadians(clawSub.get()), 0));

        //finally, move it in relation to the robot
        //NOTE: Pose3d.plus does NOT work here, as it rotates you around a piece origin, we just want to add translations
        //coral = coral.plus(new Transform3d(drivePose.getX(), drivePose.getY(), 0, Rotation3d.kZero));
        coral = new Pose3d(coral.getTranslation().plus(drivePose.getTranslation()),coral.getRotation());

        //rotate coral in relation to the robot
        coral = rotateAround(coral, drivePose.getTranslation(), drivePose.getRotation());
        coralPub.set(new Pose3d[] {coral});
    }

    /**
     * This function was supposed to be in WpiLib 2025.2.1, but was missed :(, copied here for use
     * Rotates the current pose around a point in 3D space.
     *
     * @param point The point in 3D space to rotate around.
     * @param rot The rotation to rotate the pose by.
     * @return The new rotated pose.
     */
    public Pose3d rotateAround(Pose3d currentPose, Translation3d point, Rotation3d rot) {
        return new Pose3d(currentPose.getTranslation().rotateAround(point, rot), currentPose.getRotation().rotateBy(rot));
    }
}
