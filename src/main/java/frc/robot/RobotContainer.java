package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CancelCommands;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.commands.SetHangSpeed;
import frc.robot.commands.ArmRotateTo;
import frc.robot.commands.SetLightstripColorFor;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LightStrip;
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
import frc.robot.inputs.ChassisDriveInputs;
import frc.robot.commands.FollowTag;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

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

	private final Hang hang = Constants.HangConstants.HAS_HANG
			? new RealHang(HangConstants.LEFT_MOTOR_ID, HangConstants.RIGHT_MOTOR_ID,
					HangConstants.LEFT_MOTOR_IS_INVERTED, HangConstants.RIGHT_MOTOR_IS_INVERTED,
					HangConstants.LIMIT_SWITCH_ID)
			: new DummyHang();

	private final IntakeShooter intakeShooter = Constants.IntakeShooterConstants.HAS_INTAKE ? new RealShooter(
			IntakeShooterConstants.FLYWHEEL_MOTOR_1_ID,
			IntakeShooterConstants.FLYWHEEL_MOTOR_2_ID,
			IntakeShooterConstants.INTAKE_MOTOR_ID,
			IntakeShooterConstants.INTAKE_LIMIT_SWITCH_ID 
			) : new DummyShooter();

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(
			gyro,
			swerveModuleFL, swerveModuleFR,
			swerveModuleBL, swerveModuleBR);

	private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

	private final Vision vision = new Vision(VisionConstants.CAMERA_NAME, VisionConstants.CAMERA_POSE);

	private final LightStrip lightStrip = new LightStrip(LightConstants.LED_CONTROLLER_PWM_SLOT);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser.addOption("Backward", Autos.driveAuto(drivetrain, new Translation2d(Units.feetToMeters(-7), 0)));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		configureBindings();

		setUpDriveController();
		setUpOperatorController();

		drivetrain.setFrontOffset(Rotation2d.fromDegrees(180));

		PortForwarder.add(5800, "photonvision.local", 5800);
	}

	public void toDefaultPositions() {
		drivetrain.toDefaultStates();
		arm.setArmToStartPosition();
		intakeShooter.stop();
		lightStrip.toDefaultPattern();
		// TODO Put hanger down
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final String onPortMsg = "Port " + genericHID.getPort() + ": ";

		drivetrain.removeDefaultCommand();

		final Command coopLightSignal = new SetLightstripColorFor(lightStrip, LightConstants.LED_COLOR_ORANGE, 10,
				"Coop");
		final Command amplifyLightSignal = new SetLightstripColorFor(lightStrip, LightConstants.LED_COLOR_BLUE, 10,
				"Amplify");

		final Command cancelCommand = new SequentialCommandGroup(
				new CancelCommands(drivetrain, lightStrip),
				new InstantCommand(drivetrain::toDefaultStates, drivetrain));
				new InstantCommand(lightStrip::toDefaultPattern, lightStrip);

		ChassisDriveInputs inputs = null;

		if (genericHIDType == null) {
			SmartDashboard.putString("Drive Ctrl", onPortMsg + "None");
		} else if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			SmartDashboard.putString("Drive Ctrl", onPortMsg + "Joystick");

			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					joystick::getX, -1,
					joystick::getY, -1,
					joystick::getTwist, -1,
					Constants.DriverConstants.DEAD_ZONE);

			joystick.button(1).whileTrue(Commands.startEnd(inputs::fastMode, inputs::normalMode));
			joystick.button(2).whileTrue(Commands.startEnd(inputs::slowMode, inputs::normalMode));
			joystick.button(3).onTrue(Commands.runOnce(inputs::toggleFieldRelative));

			joystick.button(4).onTrue(coopLightSignal);
			joystick.button(5).onTrue(amplifyLightSignal);
			joystick.button(6).onTrue(Commands.runOnce(vision::toggleUsing, vision));

			joystick.button(10).onTrue(cancelCommand);

		} else {
			SmartDashboard.putString("Drive Ctrl", onPortMsg + "GamePad");

			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					xbox::getLeftX, -1,
					xbox::getLeftY, -1,
					xbox::getRightX, -1,
					Constants.DriverConstants.DEAD_ZONE);

			xbox.rightTrigger().whileTrue(Commands.startEnd(inputs::fastMode, inputs::normalMode));
			xbox.leftTrigger().whileTrue(Commands.startEnd(inputs::slowMode, inputs::normalMode));

			xbox.y().onTrue(Commands.runOnce(inputs::toggleFieldRelative));
			
			xbox.povLeft().onTrue(coopLightSignal);
			xbox.povRight().onTrue(amplifyLightSignal);
			
			xbox.x().onTrue(Commands.runOnce(vision::toggleUsing, vision));

			xbox.a().whileTrue(new FollowTag(drivetrain, vision, 1, 1));

			xbox.b().onTrue(cancelCommand);
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

		final SetHangSpeed hangUp = new SetHangSpeed(hang, HangConstants.speed);
		final SetHangSpeed hangDown = new SetHangSpeed(hang, -HangConstants.speed);

		final Command cancelCommand = new SequentialCommandGroup(
				new CancelCommands(arm, intakeShooter),
				new InstantCommand(intakeShooter::stop, intakeShooter));

		if (genericHIDType == null) {
			SmartDashboard.putString("Operator Ctrl", onPortMsg + "None");

		} else if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			SmartDashboard.putString("Operator Ctrl", onPortMsg + "Joystick");
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			joystick.button(1).onTrue(Autos.intakeFromFloorStart(arm, intakeShooter, vision));
			joystick.button(1).onFalse(Autos.intakeFromFloorEnd(arm, intakeShooter, vision));

			joystick.button(2).onTrue(Autos.shootInSpeaker(arm, intakeShooter, vision));

			joystick.button(3).onTrue(Autos.dropInAmp(arm, intakeShooter, vision));

			joystick.button(4).onTrue(stowArm);
			joystick.button(6).onTrue(stowArm);

			joystick.button(7).whileTrue(Commands.startEnd(intakeShooter::eject, intakeShooter::stop, intakeShooter));

			joystick.button(8).whileTrue(hangUp);

			joystick.button(9).whileTrue(hangDown);

			joystick.button(10).onTrue(cancelCommand);

		} else {
			SmartDashboard.putString("Operator Ctrl", onPortMsg + "GamePad");
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			xbox.leftTrigger().onTrue(Autos.intakeFromFloorStart(arm, intakeShooter, vision));
			xbox.leftTrigger().onFalse(Autos.intakeFromFloorEnd(arm, intakeShooter, vision));

			xbox.rightTrigger().onTrue(Autos.shootInSpeaker(arm, intakeShooter, vision));

			xbox.rightBumper().onTrue(Autos.dropInAmp(arm, intakeShooter, vision));

			xbox.y().onTrue(stowArm);
			xbox.a().onTrue(stowArm2);

			xbox.x().whileTrue(Commands.startEnd(intakeShooter::eject, intakeShooter::stop, intakeShooter));

			xbox.povUp().whileTrue(hangUp);
			xbox.povDown().whileTrue(hangDown);

			xbox.b().onTrue(cancelCommand);
		}
	}

	/** Use this method to define your trigger->command mappings. */
	private void configureBindings() {
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