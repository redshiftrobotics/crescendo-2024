package frc.robot.commands;

import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically aim at a angle */
public class AimAtAngle extends Command {
	private final SwerveDrivetrain drivetrain;
	private final ChassisDriveInputs chassisDriveInputs;

	private final PIDController rotatePID;

	/**
	 * Create a new AimAtAngle command. Tries to constants face in angle while still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 */
	public AimAtAngle(SwerveDrivetrain drivetrain, ChassisDriveInputs chassisDriveInputs, Rotation2d direction) {
		this.drivetrain = drivetrain;

		this.chassisDriveInputs = chassisDriveInputs;

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotatePID.setSetpoint(direction.getRadians());

		addRequirements(drivetrain, chassisDriveInputs);
	}


	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {

		double rotationSpeed = 0;
		if (!rotatePID.atSetpoint()) {
			rotationSpeed = rotatePID.calculate(drivetrain.getHeading().getRadians());
		}

		double xSpeed = 0;
		double ySpeed = 0;
		boolean fieldRelative = false;
		if (chassisDriveInputs != null) {
			xSpeed = chassisDriveInputs.getX();
			ySpeed = chassisDriveInputs.getY();
			fieldRelative = chassisDriveInputs.isFieldRelative();
		}

		ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
		if (fieldRelative) desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, drivetrain.getHeading());
		desiredSpeeds.omegaRadiansPerSecond = rotationSpeed;

		drivetrain.setDesiredState(desiredSpeeds, false);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return (chassisDriveInputs == null) && (rotatePID.atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}