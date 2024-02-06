package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveDrivetrain;

/** Command to automatically drive a certain transform */
public class DriveTransform extends DriveToPoseBase {
    private final Transform2d transform;

    /**
     * Create a new DriveToPose command. Tries to drive a certain transform using the DriveToPose command.
     * 
     * <p>This drives relative to the robot, so a transform of +2x and +1y will drive 2 meters forward and 1 meter left,
     * where forward is whatever direction the robot is currently facing</p>
     * 
     * @param drivetrain the drivetrain of the robot
     * @param transform target transform to drive to, will be added to current position to get target pose
     */
    public DriveTransform(SwerveDrivetrain drivetrain, Transform2d transform) {
        super(drivetrain);
        this.transform = transform;
    }
    
    /**
     * Create a new DriveToPose command. Tries to drive a certain transform (a translation and a rotation) using the DriveToPose command
     * 
     * <p>This drives relative to the robot, so a transform of +2x and +1y will drive 2 meters forward and 1 meter left,
     * where forward is whatever direction the robot is currently facing</p>
     * 
     * @param drivetrain The drivetrain of the robot
     * @param translation Target transform to drive
     * @param translation Target rotation to drive
     */
    public DriveTransform(SwerveDrivetrain drivetrain, Translation2d translation, Rotation2d rotation) {
        this(drivetrain, new Transform2d(translation, rotation));
    }    

    @Override
    public void initialize() {
        setDesiredPosition(getPosition().plus(transform));
    }
}