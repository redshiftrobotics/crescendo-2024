package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.ChassisDriveInputs;
import frc.robot.utils.OptionButton;

/**
 * This can be the default command for the drivetrain, allowing for remote operation with a controller.
 */
public class DriverControl extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Arm arm;

	private final OptionButton preciseModeButton;
	private final OptionButton boostModeButton;
	private final OptionButton fieldRelativeButton;

	private final ChassisDriveInputs chassisDriveInputs;

	private final OptionButton raiseArmButton;
    private final OptionButton lowerArmButton;

	/**
	 * Creates a new SwerveDriveBaseControl Command.
	 *
	 * @param drivetrain       The drivetrain of the robot
	 * @param driverController The device used to control drivetrain
	 */
	public DriverControl(SwerveDrivetrain drivetrain, Arm arm, ChassisDriveInputs chassisDriveInputs,
			OptionButton preciseModeButton, OptionButton boostModeButton, OptionButton fieldRelativeButton,
			OptionButton raiseArmButton, OptionButton lowerArmButton) {

		this.chassisDriveInputs = chassisDriveInputs;

		this.preciseModeButton = preciseModeButton;
		this.boostModeButton = boostModeButton;
		this.fieldRelativeButton = fieldRelativeButton;

		this.drivetrain = drivetrain;

		this.arm = arm;

		this.raiseArmButton = raiseArmButton;
		this.lowerArmButton = lowerArmButton;

		

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

		SmartDashboard.putBoolean("ControlActive", true);
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

		final boolean isFieldRelative = fieldRelativeButton.getState();

		final int speedLevel = 1
				- preciseModeButton.getStateAsInt()
				+ boostModeButton.getStateAsInt();

		final ChassisSpeeds speeds = new ChassisSpeeds(
				speedX * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
				speedY * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
				speedRotation * DriverConstants.maxSpeedOptionsRotation[speedLevel]);


		SmartDashboard.putNumber("SpeedX", speedX);
		SmartDashboard.putNumber("SpeedY", speedY);
		SmartDashboard.putNumber("Speed", speedRotation);

		drivetrain.setDesiredState(speeds, isFieldRelative, true);

		// Display relevant data on shuffleboard.
		SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);
		SmartDashboard.putBoolean("Field Relieve", isFieldRelative);

		// Position display
		final Pose2d robotPosition = drivetrain.getPosition();

		SmartDashboard.putNumber("PoseX", robotPosition.getX());
		SmartDashboard.putNumber("PoseY", robotPosition.getY());
		SmartDashboard.putNumber("PoseDegrees", robotPosition.getRotation().getDegrees());

		// Speed and Heading
		final ChassisSpeeds currentSpeeds = drivetrain.getState();
		final double speedMetersPerSecond = Math
				.sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

		final double metersPerSecondToMilesPerHourConversion = 2.237;
		SmartDashboard.putNumber("Robot Speed", speedMetersPerSecond * metersPerSecondToMilesPerHourConversion);
		SmartDashboard.putNumber("Heading Degrees", drivetrain.getHeading().getDegrees());

		// Arm Motor
		arm.changeArmAngleDegreesBy(Double.valueOf(raiseArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod * ArmConstants.DEGREES_PER_SECOND);
        arm.changeArmAngleDegreesBy(Double.valueOf(-lowerArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod * ArmConstants.DEGREES_PER_SECOND);
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

		SmartDashboard.putBoolean("ControlActive", false);
	}
}