package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.RobotMovementConstants;

public class AutoDriveTo extends Command {
	private final SwerveDrivetrain drivetrain;

	private final PIDController xMovePID, yMovePID, rotatePID;
	private double initX, initY, goalX, goalY;

	/***
	 * Command to autonomously drive somewhere
	 * 
	 * @param subsystem   The drivetrain
	 * @param translation The translation to execute
	 */
	public AutoDriveTo(SwerveDrivetrain subsystem, Translation2d translation) {
		this.drivetrain = subsystem;

		goalX = translation.getX();
		goalY = translation.getY();

		xMovePID = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		xMovePID.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);

		yMovePID = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		yMovePID.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);

		addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {

		drivetrain.resetPosition();

		final Pose2d position = drivetrain.getPosition();

		rotatePID.setSetpoint(drivetrain.getHeading().getRadians());

		initX = position.getX();
		initY = position.getY();
	}

	@Override
	public void execute() {
		Pose2d position = this.drivetrain.getPosition();

		double targetX = position.getX() - initX;
		double targetY = position.getY() - initY;
		double targetRadians = drivetrain.getHeading().getRadians();

		double xSpeed = xMovePID.calculate(targetX, goalX);
		double ySpeed = yMovePID.calculate(targetY, goalY);

		// SmartDashboard.putNumber("Current X", targetX);
		// SmartDashboard.putNumber("Current Y", targetY);
		// SmartDashboard.putNumber("X Goal", goalX);
		// SmartDashboard.putNumber("Y Goal", goalY);

		double speedLimit = 0.5;
		double maxSpeed = Math.max(Math.abs(xSpeed), Math.abs(ySpeed));

		if (maxSpeed > speedLimit) {
			xSpeed = (xSpeed / maxSpeed) * speedLimit;
			ySpeed = (ySpeed / maxSpeed) * speedLimit;
		}

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, rotatePID.calculate(targetRadians)));
		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		boolean isEnded = xMovePID.atSetpoint() && yMovePID.atSetpoint();
		// SmartDashboard.putBoolean("xEndpoint", xMovePID.atSetpoint());
		// SmartDashboard.putBoolean("yEndpoint", yMovePID.atSetpoint());
		return isEnded;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
