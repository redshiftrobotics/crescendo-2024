package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoDriveForward extends Command {
    private final SwerveDrivetrain drivetrain;
    private long beganDriving;

    /**
     * Inject dependencies and reserve systems
     */
    public AutoDriveForward(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
    }

    /**
     * Begin moving
     */
    @Override
    public void initialize() {
        ChassisSpeeds desiredState = new ChassisSpeeds(0.5,0,0);
        this.beganDriving = System.currentTimeMillis();

        drivetrain.setDesiredState(desiredState);
    }

    // No execute() because we don't need it

    /**
     * Did we begin driving 10 or more seconds ago? If so trigger the finished condition and stop the robot.
     */
    @Override
    public boolean isFinished() {
        return (beganDriving + 10000) >= System.currentTimeMillis();
    }

    /**
     * Turn off motors when we don't want the robot to move anymore
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
