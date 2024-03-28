package frc.robot.commands.deprecated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

/** Command to automatically drive to a certain Pose on the field */
public class DriveToPose extends Command {
	private final SwerveDrivetrain drivetrain;
	private final Pose2d targetPosition;

	/**
	 * Create a new DriveToPose command. Uses setDesiredPosition on drivetrain.
	 * 
	 * <p>
	 * This drives relative to the robot starting position,
	 * so a pose of +2x and +1y will drive to the position 2 meters forward and 1
	 * meter left of whether the robot started,
	 * where forward is whatever direction the robot started in
	 * 
	 * <p>
	 * The last place the drivetrain position was reset counts as the starting
	 * position
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param position   Target pose the robot is trying to drive to
	 */
	public DriveToPose(SwerveDrivetrain drivetrain, Pose2d position) {
		this.drivetrain = drivetrain;
		this.targetPosition = position;

		addRequirements(drivetrain);
	}

	/**
	 * <p>
	 * This drives relative to the robot starting position,
	 * so a pose of +2x and +1y will drive to the position 2 meters forward and 1
	 * meter left of whether the robot started,
	 * where forward is whatever direction the robot started in
	 * 
	 * <p>
	 * The last place the drivetrain position was reset counts as the starting
	 * position
	 * 
	 * @param drivetrain  The drivetrain of the robot
	 * @param translation Target transform to drive
	 * @param rotation    Target rotation to drive
	 */
	public DriveToPose(SwerveDrivetrain drivetrain, Translation2d translation, Rotation2d rotation) {
		this(drivetrain, new Pose2d(translation, rotation));
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
