package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrivetrain;

/** Command to automatically drive a certain transform */
public class DriveTransform extends Command {
    private final SwerveDrivetrain drivetrain;
    private final Transform2d transform;

    /**
     * Create a new DriveToPose command. Tries to drive a certain transform using the DriveToPose command.
     * 
     * <p>This drives relative to the robot, so a transform of +2x and +1y will drive 2 meters forward and 1 meter left,
     * where forward is whatever direction the robot is currently facing</p>
     * 
     * @param drivetrain The drivetrain of the robot
     * @param transform Target transform to drive to, will be added to current position to get target pose
     */
    public DriveTransform(SwerveDrivetrain drivetrain, Transform2d transform) {
        this.drivetrain = drivetrain;
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
        CommandScheduler.getInstance().schedule(
            new DriveToPose(drivetrain, drivetrain.getPosition().plus(transform))
        );
    }

    @Override
    public boolean isFinished() {
        // Instantly finish since the entire command just calls another command 
        return true;
    }
}