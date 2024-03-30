package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimAtAngle;
import frc.robot.commands.ArmRotateTo;
import frc.robot.commands.AutoRotateTo;
import frc.robot.commands.Autos;
import frc.robot.commands.CancelCommands;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.commands.HangControl;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.DummyArm;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.hang.DummyHang;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.RealHang;
import frc.robot.subsystems.intake.DummyShooter;
import frc.robot.subsystems.intake.IntakeShooter;
import frc.robot.subsystems.intake.RealShooter;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
	// TODO: This is kind of gross. Next year make the controllers subsystems.

	public final CommandXboxController driverXboxController = new CommandXboxController(
			DriverConstants.DRIVER_JOYSTICK_PORT);
	public final XboxController driverXboxRaw = driverXboxController.getHID();

	public final CommandXboxController operatorXboxController = new CommandXboxController(
			DriverConstants.OPERATOR_JOYSTICK_PORT);
	public final XboxController operatorXboxRaw = operatorXboxController.getHID();

	// The robot's subsystems and commands are defined here...
	private final SwerveModule swerveModuleFL = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL,
			new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X,
					SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleFR = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
			new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X,
					-SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleBL = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
			new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X,
					SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleBR = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
			new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X,
					-SwerveDrivetrainConstants.MODULE_LOCATION_Y));

	private final AHRS gyro = new AHRS();

	/**
	 * This is the robot arm. In some situations, the robot may not have an arm, so
	 * if ArmConstants.HAS_ARM is false, a dummy class implementing the arm's API is
	 * created instead to prevent errors.
	 */
	private final Arm arm = Constants.ArmConstants.HAS_ARM ? new RealArm(
			ArmConstants.LEFT_MOTOR_ID,
			ArmConstants.RIGHT_MOTOR_ID,
			ArmConstants.RIGHT_ENCODER_ID,
			ArmConstants.ARE_MOTORS_REVERSED) : new DummyArm();

	private final Hang leftHang = Constants.HangConstants.HAS_HANG
			? new RealHang(HangConstants.LEFT_MOTOR_ID, HangConstants.LEFT_MOTOR_IS_INVERTED,
					HangConstants.LEFT_LIMIT_SWITCH_ID, "Left")
			: new DummyHang();

	private final Hang rightHang = Constants.HangConstants.HAS_HANG
			? new RealHang(HangConstants.RIGHT_MOTOR_ID, HangConstants.RIGHT_MOTOR_IS_INVERTED,
					HangConstants.RIGHT_LIMIT_SWITCH_ID, "Right")
			: new DummyHang();

	private final IntakeShooter intakeShooter = Constants.IntakeShooterConstants.HAS_INTAKE ? new RealShooter(
			IntakeShooterConstants.FLYWHEEL_MOTOR_1_ID,
			IntakeShooterConstants.FLYWHEEL_MOTOR_2_ID,
			IntakeShooterConstants.INTAKE_MOTOR_ID,
			IntakeShooterConstants.INTAKE_LIMIT_SWITCH_ID, IntakeShooterConstants.LIDAR_ID) : new DummyShooter();

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(
			gyro,
			swerveModuleFL, swerveModuleFR,
			swerveModuleBL, swerveModuleBR);

	private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

	private final SendableChooser<Alliance> teamChooser = new SendableChooser<Alliance>();

	private final Vision vision = new Vision(VisionConstants.CAMERA_NAME, VisionConstants.CAMERA_POSE);
	// private final Vision vision = null;

	// private final LightStrip lightStrip = new
	// LightStrip(LightConstants.LED_CONTROLLER_PWM_SLOT);

	private ChassisDriveInputs inputs = null;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser.addOption("Forward", Autos.startingAuto(drivetrain, arm));
		autoChooser.addOption("1+Forward", Autos.shootStartingAuto(drivetrain, arm, intakeShooter, this));
		autoChooser.setDefaultOption("2+Forward",
				Autos.shoot2StartingAuto(drivetrain, arm, intakeShooter, this));

		SmartDashboard.putData("Auto Chooser", autoChooser);

		teamChooser.setDefaultOption("Blue", Alliance.Blue);
		teamChooser.addOption("Red", Alliance.Red);
		SmartDashboard.putData("Ally", teamChooser);

		SmartDashboard.putNumber("shootOffset", 0);

		// SmartDashboard.putData("ArmUp", new ArmRotateTo(arm,
		// ArmConstants.ARM_START_DEGREES));
		// SmartDashboard.putData("ZeroYaw", new InstantCommand(drivetrain::zeroYaw));

		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		configureBindings();

		if (vision != null) {
			PortForwarder.add(5800, "photonvision.local", 5800);
		}
	}

	public void toDefaultPositions() {
		drivetrain.toDefaultStates();
		arm.setArmToStartPosition();
		intakeShooter.stop();
		// lightStrip.toDefaultPattern();
	}

	public void updateTelemetry() {
		drivetrain.updateTelemetry();
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final String onPortMsg = "Port " + genericHID.getPort() + ": ";

		drivetrain.removeDefaultCommand();

		final Command cancelCommand = new SequentialCommandGroup(
				new InstantCommand(drivetrain::toDefaultStates, drivetrain));

		if (genericHIDType == null) {
			SmartDashboard.putString("Drive Ctrl", onPortMsg + "None");
		} else if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			SmartDashboard.putString("Drive Ctrl", onPortMsg + "Joystick");

			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					joystick::getX, -1,
					joystick::getY, -1,
					joystick::getTwist, -1,
					DriverConstants.DEAD_ZONE, DriverConstants.SLEW_RATE_LIMIT_UP,
					DriverConstants.SLEW_RATE_LIMIT_DOWN);

			joystick.button(1).whileTrue(Commands.startEnd(inputs::fastMode, inputs::normalMode));
			joystick.button(2).whileTrue(Commands.startEnd(inputs::slowMode, inputs::normalMode));
			joystick.button(3).whileTrue(Commands.startEnd(inputs::enableMaxSpeedMode, inputs::disableMaxSpeedMode));
			joystick.button(4).onTrue(Commands.runOnce(inputs::toggleFieldRelative));

			if (vision != null)
				joystick.button(7).onTrue(Commands.runOnce(vision::toggleUsing, vision));

			joystick.button(10).onTrue(cancelCommand);

		} else {
			SmartDashboard.putString("Drive Ctrl", onPortMsg + "GamePad");

			inputs = new ChassisDriveInputs(
					driverXboxController::getLeftX, -1,
					driverXboxController::getLeftY, -1,
					driverXboxController::getRightX, -1,
					DriverConstants.DEAD_ZONE,
					DriverConstants.SLEW_RATE_LIMIT_UP, DriverConstants.SLEW_RATE_LIMIT_DOWN);

			driverXboxController.rightTrigger().whileTrue(Commands.startEnd(inputs::fastMode, inputs::normalMode));
			driverXboxController.leftTrigger().whileTrue(Commands.startEnd(inputs::slowMode, inputs::normalMode));

			driverXboxController.leftStick()
					.whileTrue(Commands.startEnd(inputs::enableMaxSpeedMode, inputs::disableMaxSpeedMode));

			driverXboxController.y().onTrue(Commands.runOnce(inputs::toggleFieldRelative));

			driverXboxController.a().whileTrue(new AimAtAngle(drivetrain, inputs, Rotation2d.fromDegrees(-180)));

			if (vision != null) {
				driverXboxController.x().onTrue(Commands.runOnce(vision::toggleUsing, vision));
			}

			new Trigger(() -> driverXboxRaw.getPOV() == 0)
					.onTrue(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(0), true));
			new Trigger(() -> driverXboxRaw.getPOV() == 90)
					.onTrue(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90), true));
			new Trigger(() -> driverXboxRaw.getPOV() == 180)
					.onTrue(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(180), true));
			new Trigger(() -> driverXboxRaw.getPOV() == 270)
					.onTrue(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(270), true));

			driverXboxController.leftBumper().onTrue(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(300)));
			driverXboxController.rightBumper().onTrue(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(60)));

			driverXboxController.b().onTrue(cancelCommand);
		}

		if (inputs != null)
			drivetrain.setDefaultCommand(new ChassisRemoteControl(drivetrain, inputs));
	}

	public void setUpOperatorController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.OPERATOR_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final String onPortMsg = "Port " + genericHID.getPort() + ": ";

		// final Command armUp = new ArmRotateBy(arm, +ArmConstants.DEGREES_PER_SECOND);
		// final Command armDown = new ArmRotateBy(arm,
		// -ArmConstants.DEGREES_PER_SECOND);

		final Command stowArm = new ArmRotateTo(arm, ArmConstants.ARM_STOW_DEGREES);
		final Command stowArm2 = new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES);

		leftHang.removeDefaultCommand();
		rightHang.removeDefaultCommand();

		final Command cancelCommand = new SequentialCommandGroup(
				new InstantCommand(intakeShooter::stop, intakeShooter),
				new CancelCommands(arm, intakeShooter),
				new InstantCommand(intakeShooter::stop, intakeShooter));

		if (genericHIDType == null) {
			SmartDashboard.putString("Operator Ctrl", onPortMsg + "None");

		} else if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			SmartDashboard.putString("Operator Ctrl", onPortMsg + "Joystick");
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			joystick.button(10).onTrue(cancelCommand);

		} else {
			SmartDashboard.putString("Operator Ctrl", onPortMsg + "GamePad");

			operatorXboxController.leftTrigger().onTrue(Autos.intakeFromFloorStart(arm, intakeShooter));
			operatorXboxController.leftTrigger().onFalse(Autos.intakeFromFloorEnd(arm, intakeShooter));

			operatorXboxController.rightBumper()
					.onTrue(Autos.dropInAmp(drivetrain, arm, intakeShooter, vision, teamChooser)
							.finallyDo(intakeShooter::stop));

			operatorXboxController.rightTrigger()
					.onTrue(Autos.shootInSpeaker(drivetrain, arm, intakeShooter, vision, teamChooser, this)
							.finallyDo(intakeShooter::stop));

			operatorXboxController.y().onTrue(stowArm);
			operatorXboxController.a().onTrue(stowArm2);

			operatorXboxController.x()
					.whileTrue(Commands.startEnd(intakeShooter::eject, intakeShooter::stop, intakeShooter));

			leftHang.setDefaultCommand(new HangControl(leftHang, operatorXboxController::getLeftY));
			rightHang.setDefaultCommand(new HangControl(rightHang, operatorXboxController::getRightY));

			new Trigger(() -> operatorXboxRaw.getPOV() == 0)
					.onTrue(new InstantCommand(() -> SmartDashboard.putNumber("shootOffset",
							SmartDashboard.getNumber("shootOffset", 0) + 1)));
			new Trigger(() -> operatorXboxRaw.getPOV() == 180)
					.onTrue(new InstantCommand(() -> SmartDashboard.putNumber("shootOffset",
							SmartDashboard.getNumber("shootOffset", 0) - 1)));

			operatorXboxController.b().onTrue(cancelCommand);
		}
	}

	/** Use this method to define your trigger->command mappings. */
	public void configureBindings() {
		setUpDriveController();
		setUpOperatorController();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
};