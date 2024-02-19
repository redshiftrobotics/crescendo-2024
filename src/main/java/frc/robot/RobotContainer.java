package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.commands.Autos;
import frc.robot.commands.ArmRemoteControl;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Autos;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.inputs.ChassisDriveInputs;
import frc.robot.inputs.OptionButtonInput;
import frc.robot.inputs.OptionButtonInput.ActivationMode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
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

    private final Arm arm = new Arm(
        ArmConstants.LEFT_MOTOR_ID,
        ArmConstants.RIGHT_MOTOR_ID,
        ArmConstants.RIGHT_ENCODER_ID,
		ArmConstants.ARE_MOTORS_REVERSED);

	private final AHRS gyro = new AHRS(I2C.Port.kOnboard);

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR,
			swerveModuleBL, swerveModuleBR);

	private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

	private final Vision vision = new Vision(VisionConstants.CAMERA_NAME, Constants.VisionConstants.CAMERA_POSE);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser.setDefaultOption("Testing Auto", Autos.testingAuto(drivetrain));
		autoChooser.addOption("Follow Tag", Autos.tagFollowAuto(drivetrain, vision, 1));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();

		setUpDriveController();

		PortForwarder.add(5800, "photonvision.local", 5800);
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final CommandJoystick operatorJoystick = new CommandJoystick(1);

		SmartDashboard.putString("Drive Controller", genericHIDType.toString());
		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		drivetrain.removeDefaultCommand();

		ArmRemoteControl armInputs;
		ChassisDriveInputs inputs;
		OptionButtonInput preciseModeButton, boostModeButton, fieldRelativeButton;

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());
			
			inputs = new ChassisDriveInputs(
					joystick::getY, -1,
					joystick::getX, -1,
					joystick::getTwist, -1,
					Constants.DriverConstants.DEAD_ZONE);

			armInputs = new ArmRemoteControl(arm,
					new OptionButtonInput(joystick,11, ActivationMode.HOLD), 
					new OptionButtonInput(joystick,12, ActivationMode.HOLD), 

					new OptionButtonInput(joystick, 4, ActivationMode.HOLD), 
					new OptionButtonInput(joystick, 5, ActivationMode.HOLD), 
					new OptionButtonInput(joystick, 6, ActivationMode.HOLD)
					);

			preciseModeButton = new OptionButtonInput(joystick, 2, ActivationMode.TOGGLE);
			boostModeButton = new OptionButtonInput(joystick, 1, ActivationMode.HOLD);
			fieldRelativeButton = new OptionButtonInput(joystick, 3, ActivationMode.TOGGLE);


			operatorJoystick.button(4).onTrue(Commands.run(() -> arm.setArmToAmpPosition(), arm));
			operatorJoystick.button(5).onTrue(Commands.run(() -> arm.setArmToIntakePosition(), arm));
			operatorJoystick.button(6).onTrue(Commands.run(() -> arm.setArmToSpeakerPosition(), arm));

			joystick.button(9).onTrue(Commands.run(drivetrain::brakeMode, drivetrain));
			joystick.button(10).onTrue(Commands.run(drivetrain::toDefaultStates, drivetrain));
		} else {
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					xbox::getLeftY, +1,
					xbox::getLeftX, +1,
					xbox::getRightX, -1,
					Constants.DriverConstants.DEAD_ZONE);

			armInputs = new ArmRemoteControl(arm,
					new OptionButtonInput(xbox::rightBumper, ActivationMode.HOLD), 
					new OptionButtonInput(xbox::leftBumper, ActivationMode.HOLD), 

					new OptionButtonInput(xbox::povLeft, ActivationMode.HOLD), 
					new OptionButtonInput(xbox::povRight, ActivationMode.HOLD), 
					new OptionButtonInput(xbox::povDown, ActivationMode.HOLD)
					);


			

			preciseModeButton = new OptionButtonInput(xbox::b, ActivationMode.TOGGLE);
			boostModeButton = new OptionButtonInput(xbox::leftStick, ActivationMode.HOLD);
			fieldRelativeButton = new OptionButtonInput(xbox::povUp, ActivationMode.TOGGLE);
		}

		drivetrain.setDefaultCommand(new ChassisRemoteControl(drivetrain, inputs, preciseModeButton, boostModeButton, fieldRelativeButton));
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
