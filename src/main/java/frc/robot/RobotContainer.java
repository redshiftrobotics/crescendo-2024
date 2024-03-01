package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.AimAtTag;
import frc.robot.commands.ArmRotateTo;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.commands.SetLightstripColor;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.ArmInterface;
import frc.robot.subsystems.arm.DummyArm;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.inputs.ChassisDriveInputs;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
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
	private final ArmInterface arm = Constants.ArmConstants.HAS_ARM ? new RealArm(
			ArmConstants.LEFT_MOTOR_ID,
			ArmConstants.RIGHT_MOTOR_ID,
			ArmConstants.RIGHT_ENCODER_ID,
			ArmConstants.ARE_MOTORS_REVERSED) : new DummyArm();

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR,
			swerveModuleBL, swerveModuleBR);

	private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

	private final Vision vision = new Vision(VisionConstants.CAMERA_NAME, VisionConstants.CAMERA_POSE);

	private final LightStrip lightStrip = new LightStrip(new AddressableLED(LightConstants.LED_CONTROLLER_PWM_SLOT));

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser.addOption("Rotate by 90", Autos.rotateTestAuto(drivetrain, 90, false));
		autoChooser.addOption("Forward", Autos.driveAuto(drivetrain, +1));
		autoChooser.addOption("Backward", Autos.driveAuto(drivetrain, -1));
		autoChooser.addOption("Make LEDs blue", new SetLightstripColor(lightStrip, 0, 0, 200));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		configureBindings();

		setUpDriveController();

		PortForwarder.add(5800, "photonvision.local", 5800);
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		SmartDashboard.putString("Drive Controller", genericHIDType.toString());

		drivetrain.removeDefaultCommand();

		ChassisDriveInputs inputs;

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			inputs = new ChassisDriveInputs(
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
		} else {
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					xbox::getLeftX, -1,
					xbox::getLeftY, -1,
					xbox::getRightX, -1,
					Constants.DriverConstants.DEAD_ZONE);

			// xbox.povDown().whileTrue(Commands.run(drivetrain::brakeMode, drivetrain));
			// xbox.povLeft().whileTrue(Commands.run(drivetrain::toDefaultStates,
			// drivetrain));

			xbox.b().onTrue(Commands.runOnce(inputs::decreaseSpeedLevel));
			xbox.povUp().onTrue(Commands.runOnce(inputs::increaseSpeedLevel));
			xbox.button(3).onTrue(Commands.runOnce(inputs::toggleFieldRelative));

			xbox.a().whileTrue(new AimAtTag(drivetrain, vision, 1, inputs));
		}

		drivetrain.setDefaultCommand(new ChassisRemoteControl(drivetrain, inputs));
	}

	public void setUpOperatorController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final ArmRotateTo armToIntake = new ArmRotateTo(arm, ArmConstants.ARM_INTAKE_DEGREES);
		final ArmRotateTo armToAmp = new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES);
		final ArmRotateTo armToSpeaker = new ArmRotateTo(arm, ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES);

		SmartDashboard.putString("Operator Controller", genericHIDType.toString());

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			joystick.button(4).onTrue(armToIntake);
			joystick.button(5).onTrue(armToAmp);
			joystick.button(6).onTrue(armToSpeaker);
		} else {
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			xbox.rightTrigger().onTrue(armToIntake);
			xbox.leftTrigger(5).onTrue(armToSpeaker);
			xbox.povDown().onTrue(armToAmp);

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
}
