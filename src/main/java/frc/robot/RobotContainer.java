package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.SwerveRemoteOperation.SwerveDriveJoystickControl;
import frc.robot.commands.SwerveRemoteOperation.SwerveDriveXboxControl;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import java.util.List;
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
            new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X, SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleFR = new SwerveModule(
            SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
            new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X, -SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleBL = new SwerveModule(
            SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
            new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X, SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleBR = new SwerveModule(
            SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
            new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X, -SwerveDrivetrainConstants.MODULE_LOCATION_Y));

    private final AHRS gyro = new AHRS(I2C.Port.kMXP);

    private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR, swerveModuleBL, swerveModuleBR);

    // Create joysticks
    private final CommandJoystick driverJoystick = new CommandJoystick(DriverConstants.DRIVER_JOYSTICK_PORT);
    // private final CommandJoystick operatorJoystick = new CommandJoystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);




    //private final CommandXboxController xboxController = new CommandXboxController(0);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        SwerveDriveJoystickControl control = new SwerveDriveJoystickControl(drivetrain, driverJoystick);
        //SwerveDriveXboxControl control = new SwerveDriveXboxControl(drivetrain, xboxController);
        drivetrain.setDefaultCommand(control);

        // Configure the trigger bindings
        configureBindings();
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
        // An example command will be run in autonomous

        //Trajectory Config
        final TrajectoryConfig exampleConfig = new TrajectoryConfig(AutoConstants.kMaxAutoVelocitySpeedMetersPerSecond,AutoConstants.kMaxAutoRotationSpeedMetersPerSecond).setKinematics(drivetrain.getKinematics());
        //Example Trajectory (1 meter forward then backward)
        final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(Math.PI*0.5)),List.of(new Translation2d(0,1)),new Pose2d(0,0,new Rotation2d(Math.PI *0.5)), exampleConfig);
        //Profiled PID Controller for trajectory rotation
        final ProfiledPIDController rotationPidController = new ProfiledPIDController(AutoConstants.kAngularControllerP, 0, 0, AutoConstants.kRotationControllerConstraints);
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        //SwerveControllerCommand Test (Trajectory Auto Drive)
        final SwerveControllerCommand trajectoryTestCommand = new SwerveControllerCommand(exampleTrajectory, drivetrain::getPosition, drivetrain.getKinematics(), new PIDController(AutoConstants.kVelocityControllerP, 0, 0), new PIDController(AutoConstants.kVelocityControllerP, 0, 0), rotationPidController, drivetrain::setDesiredStateDriveSwerveModuleStates, drivetrain);

        return Commands.sequence(new InstantCommand(() -> drivetrain.resetPosition()), trajectoryTestCommand, new InstantCommand(()-> drivetrain.setDesiredState(new ChassisSpeeds(0,0,0))));
    }
}
