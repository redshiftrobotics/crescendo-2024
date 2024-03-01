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

	private final PIDController xMovePID, yMovePID;
	private double initX, initY, goalX, goalY;

	/***
	 * Command to autonomously drive somewhere
	 * @param subsystem The drivetrain
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

		addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {

		drivetrain.resetPosition();

		final Pose2d position = drivetrain.getPosition();

		initX = position.getX();
		initY = position.getY();
	}

	@Override
	public void execute() {
		Pose2d position = this.drivetrain.getPosition();

		double targetX = position.getX() - initX;
		double targetY = position.getY() - initY;

		double xSpeed = xMovePID.calculate(targetX, goalX);
		double ySpeed = yMovePID.calculate(targetY, goalY);

		SmartDashboard.putNumber("Speed X", xSpeed);
		SmartDashboard.putNumber("Speed Y", ySpeed);
		double speedLimit = RobotMovementConstants.MAX_TRANSLATION_SPEED;
		double maxSpeed = Math.max(Math.abs(xSpeed), Math.abs(ySpeed));

		if (maxSpeed > speedLimit) {
			xSpeed = (xSpeed / maxSpeed) * speedLimit;
			ySpeed = (ySpeed / maxSpeed) * speedLimit;
		}

		SmartDashboard.putNumber("Scaled X", xSpeed);
		SmartDashboard.putNumber("Scaled Y", ySpeed);

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, 0));
		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return xMovePID.atSetpoint() && yMovePID.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
