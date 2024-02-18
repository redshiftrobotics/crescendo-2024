package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotMovementConstants;

public class AutoDriveTo extends Command {
	private final SwerveDrivetrain drivetrain;

	private final PIDController xMovePID, yMovePID;
	private double initX, initY, goalX, goalY;

	private boolean xOnlyMode;

	/***
	 * Command to autonomously drive somewhere
	 * @param subsystem The drivetrain
	 * @param translation The translation to execute
	 */
	public AutoDriveTo(SwerveDrivetrain subsystem, Translation2d translation) {
		this.drivetrain = subsystem;

		goalX = translation.getX();
		goalY = translation.getY();

		xOnlyMode = Math.abs(goalX) > Math.abs(goalY);

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

		double x = position.getX() - initX;
		double y = position.getY() - initY;

		SmartDashboard.putNumber("PoseY", position.getY());
		SmartDashboard.putNumber("PoseX", position.getX());
		SmartDashboard.putNumber("PoseDegrees", position.getRotation().getDegrees());

		double xSpeed = xMovePID.calculate(x, goalX);
		double ySpeed = yMovePID.calculate(y, goalY);

		if (xOnlyMode)
			ySpeed = 0;
		else
			xSpeed = 0;

		// TEMP FIX: LEAVE HERE UNTIL BUMPERS!!!
		if (Math.abs(xSpeed) > 0.5) {
			xSpeed = 0.5 * Math.signum(xSpeed);
		}

		final ChassisSpeeds speeds = new ChassisSpeeds(
				xSpeed,
				ySpeed,
				0);

		drivetrain.setDesiredState(speeds);
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
