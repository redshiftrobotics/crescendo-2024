package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.commands.Autos;
import frc.robot.commands.ArmControl;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.ChassisDriveInputs;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
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
        ArmConstants.RIGHT_ENCODER_ID);

	private final AHRS gyro = new AHRS(I2C.Port.kOnboard);

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR,
			swerveModuleBL, swerveModuleBR);

	private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser.setDefaultOption("Testing Auto", Autos.testingAuto(drivetrain));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();

		setUpDriveController();
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		SmartDashboard.putString("Drive Controller", genericHIDType.toString());
		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		drivetrain.removeDefaultCommand();

		DriverControl control;
		ArmControl armcontrol;

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());
			control = new DriverControl(drivetrain,

				new ChassisDriveInputs(
					joystick::getX, joystick::getY, joystick::getTwist,
					-1, -1, Constants.DriverConstants.DEAD_ZONE),

				new OptionButton(joystick, 2, ActivationMode.TOGGLE),
				new OptionButton(joystick, 1, ActivationMode.HOLD),
				new OptionButton(joystick, 3, ActivationMode.TOGGLE)

			);

			armcontrol = new ArmControl(arm,

				new OptionButton(joystick,11,ActivationMode.HOLD),
				new OptionButton(joystick, 12, ActivationMode.HOLD),

				new OptionButton(joystick::povLeft, ActivationMode.HOLD),
				new OptionButton(joystick::povRight, ActivationMode.HOLD),
				new OptionButton(joystick::povDown, ActivationMode.HOLD)
			);
			
			joystick.button(4).onTrue(Commands.run(drivetrain::brakeMode, drivetrain));

		} else {
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());
			control = new DriverControl(drivetrain, 

				new ChassisDriveInputs(
					xbox::getLeftX, xbox::getLeftY, xbox::getRightX,
					+1, -1, Constants.DriverConstants.DEAD_ZONE),

				new OptionButton(xbox::b, ActivationMode.TOGGLE),
				new OptionButton(xbox::leftStick, ActivationMode.HOLD),
				new OptionButton(xbox::povUp, ActivationMode.TOGGLE)

			);

			armcontrol = new ArmControl(arm,

				new OptionButton(xbox::rightBumper,ActivationMode.HOLD),
				new OptionButton(xbox::leftBumper, ActivationMode.HOLD),

				new OptionButton(xbox::povLeft, ActivationMode.HOLD),
				new OptionButton(xbox::povRight, ActivationMode.HOLD),
				new OptionButton(xbox::povDown, ActivationMode.HOLD)
			);


		}

		drivetrain.setDefaultCommand(control);
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
