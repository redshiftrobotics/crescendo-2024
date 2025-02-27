package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This can be the default command for the drivetrain.
 * It should allow the driver to control the robot, as well displaying relevant
 * driver data to SmartDashboard
 */
public class ChassisRemoteControl extends Command {
	protected final SwerveDrivetrain drivetrain;

	private final ChassisDriveInputs chassisDriveInputs;

	/**
	 * Creates a new SwerveDriveBaseControl Command.
	 */
	public ChassisRemoteControl(SwerveDrivetrain drivetrain, ChassisDriveInputs chassisDriveInputs) {

		this.chassisDriveInputs = chassisDriveInputs;

		this.drivetrain = drivetrain;

		// Tell the command schedular we are using the drivetrain
		addRequirements(drivetrain);
	}

	/**
	 * The initial subroutine of a command. Called once when the command is
	 * initially scheduled.
	 * Puts all swerve modules to the default state, staying still and facing
	 * forwards.
	 */
	@Override
	public void initialize() {
		drivetrain.toDefaultStates();

		SmartDashboard.putBoolean("Control Active", true);
	}

	/**
	 * The main body of a command. Called repeatedly while the command is scheduled
	 * (Every 20 ms).
	 */
	@Override
	public void execute() {
		final double speedX = chassisDriveInputs.getX();
		final double speedY = chassisDriveInputs.getY();

		final double speedRotation = chassisDriveInputs.getRotation();

		final boolean isFieldRelative = chassisDriveInputs.isFieldRelative();

		final ChassisSpeeds speeds = new ChassisSpeeds(
				speedX,
				speedY,
				speedRotation);

		drivetrain.setDesiredState(speeds, isFieldRelative);
	}

	/**
	 * Whether the command has finished. Once a command finishes, the scheduler will
	 * call its end() method and un-schedule it.
	 * Always return false since we never want to end in this case.
	 */
	@Override
	public boolean isFinished() {
		return false;
	}

	/**
	 * The action to take when the command ends. Called when either the command
	 * finishes normally, or when it interrupted/canceled.
	 * Should only happen in this case if we get interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();

		SmartDashboard.putBoolean("Control Active", false);
	}
}