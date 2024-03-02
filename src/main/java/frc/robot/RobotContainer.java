package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CancelCommands;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.commands.FollowTag;
import frc.robot.commands.AimAtTag;
import frc.robot.commands.ArmRotateBy;
import frc.robot.commands.ArmRotateTo;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.commands.SetHangSpeed;
import frc.robot.commands.SetLightstripColor;
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

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
			IntakeShooterConstants.FLYWHEEL_MOTOR_LEFT_ID,
			IntakeShooterConstants.FLYWHEEL_MOTOR_RIGHT_ID,
			IntakeShooterConstants.INTAKE_MOTOR_LEFT_ID,
			IntakeShooterConstants.INTAKE_MOTOR_RIGHT_ID) : new DummyShooter();

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
		autoChooser.addOption("Rotate by 90", Autos.rotateTestAuto(drivetrain, 90, false));
		autoChooser.addOption("Forward", Autos.driveAuto(drivetrain, +1));
		autoChooser.addOption("Backward", Autos.driveAuto(drivetrain, -1));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		configureBindings();

		setUpDriveController();
		setUpOperatorController();

		PortForwarder.add(5800, "photonvision.local", 5800);
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final Command cancelCommand = new CancelCommands(drivetrain, arm, intakeShooter);

		drivetrain.removeDefaultCommand();

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			SmartDashboard.putString("Drive Ctrl", "Joystick");

			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			ChassisDriveInputs inputs = new ChassisDriveInputs(
					joystick::getX, -1,
					joystick::getY, -1,
					joystick::getTwist, -1,
					Constants.DriverConstants.DEAD_ZONE);

			joystick.button(1).onTrue(Commands.runOnce(inputs::increaseSpeedLevel));
			// joystick.button(1).onFalse(Commands.runOnce(inputs::decreaseSpeedLevel));

			joystick.button(2).onTrue(Commands.runOnce(inputs::decreaseSpeedLevel));
			// joystick.button(2).onFalse(Commands.runOnce(inputs::increaseSpeedLevel));

			joystick.button(3).onTrue(Commands.runOnce(inputs::toggleFieldRelative));

			// joystick.button(9).onTrue(Commands.run(drivetrain::brakeMode, drivetrain));
			// joystick.button(10).onTrue(Commands.run(drivetrain::toDefaultStates,
			// drivetrain));

			drivetrain.setDefaultCommand(new ChassisRemoteControl(drivetrain, inputs));

		} else {
			SmartDashboard.putString("Drive Ctrl", "GamePad");

			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			ChassisDriveInputs inputs = new ChassisDriveInputs(
					xbox::getLeftX, -1,
					xbox::getLeftY, -1,
					xbox::getRightX, -1,
					Constants.DriverConstants.DEAD_ZONE);

			xbox.leftBumper().whileTrue(Commands.run(drivetrain::brakeMode, drivetrain));
			xbox.rightBumper().whileTrue(Commands.run(drivetrain::toDefaultStates, drivetrain));

			xbox.povDown().onTrue(Commands.runOnce(inputs::decreaseSpeedLevel));
			xbox.povUp().onTrue(Commands.runOnce(inputs::increaseSpeedLevel));
			xbox.y().onTrue(Commands.runOnce(inputs::toggleFieldRelative));

			xbox.b().onTrue(cancelCommand);

			xbox.a().whileTrue(new AimAtTag(drivetrain, vision, 1, inputs));
			xbox.x().whileTrue(new FollowTag(drivetrain, vision, 1, AutoConstants.PREFERRED_TAG_DISTANCE));

			drivetrain.setDefaultCommand(new ChassisRemoteControl(drivetrain, inputs));
		}
	}

	public void toDefaultPositions() {
		drivetrain.toDefaultStates();
		arm.setArmToStartPosition();
	}

	public void setUpOperatorController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.OPERATOR_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final Command armToIntake = new ArmRotateTo(arm, ArmConstants.ARM_INTAKE_DEGREES);
		final Command armToAmp = new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES);
		final Command armToSpeaker = new ArmRotateTo(arm, ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES);
		final Command armToStart = new ArmRotateTo(arm, ArmConstants.ARM_START);
		final Command armToDrive = new ArmRotateTo(arm, ArmConstants.ARM_DRIVE);

		final Command armUp = new ArmRotateBy(arm, +ArmConstants.DEGREES_PER_SECOND * TimedRobot.kDefaultPeriod);
		final Command armDown = new ArmRotateBy(arm, -ArmConstants.DEGREES_PER_SECOND * TimedRobot.kDefaultPeriod);

		final Command intake = Commands.runOnce(intakeShooter::intake, intakeShooter);
		final Command intakeReverse = Commands.runOnce(intakeShooter::intakeReverse, intakeShooter);
		final Command intakeStop = Commands.runOnce(intakeShooter::stopIntake, intakeShooter);

		final Command startFlyWheel = Commands.runOnce(intakeShooter::startFlyWheels, intakeShooter);
		final Command reverseFlyWheel = Commands.runOnce(intakeShooter::reverseFlywheel, intakeShooter);
		final Command stopFlyWheel = Commands.runOnce(intakeShooter::stopFlywheels, intakeShooter);

		final Command amplifyLightSignal = new SetLightstripColor(lightStrip, LightConstants.LED_COLOR_BLUE);

		// because coop light is orange
		final Command coopLightSignal = new SetLightstripColor(lightStrip, LightConstants.LED_COLOR_ORANGE);

		final Command cancelCommand = new CancelCommands(drivetrain, arm, intakeShooter);
    
		final SetHangSpeed hangStop = new SetHangSpeed(hang, 0);
		final SetHangSpeed hangForwardSpeed = new SetHangSpeed(hang, HangConstants.speed);
		final SetHangSpeed hangBackwardSpeed = new SetHangSpeed(hang, -HangConstants.speed);

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			SmartDashboard.putString("Operator Ctrl", "Joystick");
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			joystick.button(4).onTrue(armToIntake);
			joystick.button(5).onTrue(armToAmp);
			joystick.button(6).onTrue(armToSpeaker);

			joystick.button(7).or(joystick.button(8)).whileFalse(hangStop);
			joystick.button(7).onTrue(hangForwardSpeed);
			joystick.button(8).onTrue(hangBackwardSpeed);
      
			joystick.button(9).onTrue(amplifyLightSignal);
			joystick.button(10).onTrue(coopLightSignal);
		} else {
			SmartDashboard.putString("Operator Ctrl", "GamePad");
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			// xbox.rightBumper().or(xbox.leftBumper()).whileFalse(hangStop);
			// xbox.rightBumper().onTrue(hangForwardSpeed);
			// xbox.leftBumper().onTrue(hangBackwardSpeed);

			xbox.leftTrigger().onTrue(startFlyWheel);
			xbox.leftTrigger().onFalse(stopFlyWheel);

			xbox.leftStick().onTrue(reverseFlyWheel);
			xbox.leftStick().onFalse(stopFlyWheel);

			xbox.rightTrigger().onTrue(intakeReverse);
			xbox.rightTrigger().onFalse(intakeStop);

			xbox.leftBumper().onTrue(armToIntake);

			xbox.rightBumper().onTrue(armToSpeaker);

			xbox.y().onTrue(intake);
			xbox.y().onFalse(intakeStop);

			xbox.pov(270).onTrue(armToStart);
			xbox.pov(90).onTrue(armToDrive);

			xbox.a().onTrue(armToAmp);

			// xbox.povLeft().onTrue(amplifyLightSignal);
			// xbox.povRight().onTrue(coopLightSignal);

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