package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;

/** Command to automatically aim at a angle */
public class AimAtAngle extends Command {
	private final SwerveDrivetrain drivetrain;
	private final ChassisDriveInputs chassisDriveInputs;

	private final PIDController rotatePID;

	/**
	 * Create a new AimAtAngle command. Tries to constants face in angle while still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 */
	public AimAtAngle(SwerveDrivetrain drivetrain, ChassisDriveInputs chassisDriveInputs, Rotation2d direction) {
		this.drivetrain = drivetrain;

		this.chassisDriveInputs = chassisDriveInputs;

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setSetpoint(direction.getRadians());

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		rotatePID.reset();
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {

		double rotationSpeed = rotatePID.calculate(drivetrain.getHeading().getRadians());

		double xSpeed = 0;
		double ySpeed = 0;
		boolean fieldRelative = false;
		if (chassisDriveInputs != null) {
			xSpeed = chassisDriveInputs.getX();
			ySpeed = chassisDriveInputs.getY();
			fieldRelative = chassisDriveInputs.isFieldRelative();
		}

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), fieldRelative);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.setDesiredState(new ChassisSpeeds());
	}
}