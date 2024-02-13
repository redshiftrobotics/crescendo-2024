package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.ChassisDriveInputs;
import frc.robot.utils.OptionButton;
import java.util.function.Supplier;

/**
 * This can be the default command for the drivetrain, allowing for remote
 * operation with a controller
 */
public class DriverControl extends Command {
	protected final SwerveDrivetrain drivetrain;

	private final OptionButton preciseModeButton;
	private final OptionButton boostModeButton;
	private final OptionButton fieldRelativeButton;

	private final ChassisDriveInputs chassisDriveInputs;

	private final Supplier<Double> speedX;
	private final Supplier<Double> speedY;
	private final Supplier<Double> speedRotation;
	private final Supplier<Boolean> isFieldRelative;
	private final Supplier<Integer> speedLevel;
	private final Supplier<ChassisSpeeds> speeds;

	private ShuffleboardTab driverTab;
	private GenericEntry widgetSpeedX;
	private GenericEntry widgetSpeedY;
	private GenericEntry widgetSpeed;
	private GenericEntry widgetSpeedMode;
	private GenericEntry widgetFieldRelative;
	private GenericEntry widgetPoseX;
	private GenericEntry widgetPoseY;
	private GenericEntry widgetPoseDegrees;
	private ShuffleboardTab developerTab;
	private GenericEntry widgetTotalSpeed;
	private GenericEntry widgetHeading;

	/**
	 * Creates a new DriverControl Command.
	 */
	public DriverControl(SwerveDrivetrain drivetrain, ChassisDriveInputs chassisDriveInputs,
			OptionButton preciseModeButton, OptionButton boostModeButton, OptionButton fieldRelativeButton) {
		this.chassisDriveInputs = chassisDriveInputs;

		this.preciseModeButton = preciseModeButton;
		this.boostModeButton = boostModeButton;
		this.fieldRelativeButton = fieldRelativeButton;

		this.drivetrain = drivetrain;

		speedX = chassisDriveInputs::getX;
		speedY = chassisDriveInputs::getY;

		speedRotation = chassisDriveInputs::getRotation;
		isFieldRelative = fieldRelativeButton::getState;

		speedLevel = () -> 1
				- preciseModeButton.getStateAsInt()
				+ boostModeButton.getStateAsInt();

		speeds = () -> new ChassisSpeeds(
				speedX.get() * DriverConstants.maxSpeedOptionsTranslation[speedLevel.get()],
				speedY.get() * DriverConstants.maxSpeedOptionsTranslation[speedLevel.get()],
				speedRotation.get() * DriverConstants.maxSpeedOptionsRotation[speedLevel.get()]);

		addRequirements(drivetrain);
	}

	/**
	 * Puts all swerve modules to the default state, staying still and facing
	 * forwards. Also turns control activity indicator green.
	 */
	@Override
	public void initialize() {
		drivetrain.toDefaultStates();

		driverTab = Shuffleboard.getTab("Driver Control");
		driverTab.add("Control Active", true);
		widgetTotalSpeed = driverTab.add("Robot Speed", 0).getEntry();
		widgetHeading = driverTab.add("Heading", 0).getEntry();

		developerTab = Shuffleboard.getTab("Debug");
		widgetSpeedX = developerTab.add("SpeedX", speedX.get()).getEntry();
		widgetSpeedY = developerTab.add("SpeedY", speedY.get()).getEntry();
		widgetSpeed = developerTab.add("SpeedRot", speedRotation.get()).getEntry();
		widgetSpeedMode = developerTab.add("Speed Mode", "-").getEntry();
		widgetFieldRelative = developerTab.add("Field Relative", false).getEntry();
		widgetPoseX = developerTab.add("PoseX", 0).getEntry();
		widgetPoseY = developerTab.add("PoseY", 0).getEntry();
		widgetPoseDegrees = developerTab.add("PoseDegrees", 0).getEntry();
	}

	@Override
	public void execute() {
		drivetrain.setDesiredState(speeds.get(), isFieldRelative.get(), true);

		widgetSpeed.setDouble(speedRotation.get());
		widgetSpeedX.setDouble(speedX.get());
		widgetSpeedY.setDouble(speedY.get());

		// Display relevant data on shuffleboard.
		widgetSpeedMode.setString(DriverConstants.maxSpeedOptionsNames[speedLevel.get()]);
		widgetFieldRelative.setBoolean(isFieldRelative.get());

		// Position display
		final Pose2d robotPosition = drivetrain.getPosition();

		widgetPoseX.setDouble(robotPosition.getX());
		widgetPoseY.setDouble(robotPosition.getY());
		widgetPoseDegrees.setDouble(robotPosition.getRotation().getDegrees());

		// Speed and Heading
		final ChassisSpeeds currentSpeeds = drivetrain.getState();
		final double speedMetersPerSecond = Math
				.sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

		final double metersPerSecondToMilesPerHourConversion = 2.237;
		widgetTotalSpeed.setDouble(speedMetersPerSecond * metersPerSecondToMilesPerHourConversion);
		widgetHeading.setDouble(drivetrain.getHeading().getDegrees());
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

		// turn red
	}
}