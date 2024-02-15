package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

/** Command to automatically drive to a certain Pose on the field */
public class DriveToPose extends Command {
	private final SwerveDrivetrain drivetrain;
	private final Pose2d targetPosition;

    /**
     * Create a new DriveToPose command. Uses setDesiredPosition on drivetrain.
     * 
     * <p>This drives relative to the robot starting position,
	 * so a pose of +2x and +1y will drive to the position 2 meters forward and 1 meter left of whether the robot started,
     * where forward is whatever direction the robot started in</p>
     * 
     * <p>The last place the drivetrain position was reset counts as the starting position</p>
     * 
     * @param drivetrain the drivetrain of the robot
     * @param targetPosition the pose the robot is trying to achieve
     */
    public DriveToPose(SwerveDrivetrain drivetrain, Pose2d targetPosition) {
		this.drivetrain = drivetrain;
		this.targetPosition = targetPosition;
    }

	@Override
	public void initialize() {
		drivetrain.setDesiredPosition(targetPosition);
	}

	@Override
	public boolean isFinished() {
		return drivetrain.isAtDesiredPosition();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.clearDesiredPosition();
		drivetrain.stop();
	}
}
