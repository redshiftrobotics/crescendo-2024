package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotMovementConstants;

// How to make Command (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/commands/commands.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Code documentations https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html 

public class AutoDriveTo extends Command {
	private final SwerveDrivetrain drivetrain;

	private final PIDController xMovePID, yMovePID;
	private double initX, initY, goalX, goalY;

	private double atSetpointCounter = 0;

	private boolean xOnlyMode;

	public AutoDriveTo(SwerveDrivetrain subsystem, Translation2d translation) {
		this.drivetrain = subsystem;

		goalX = translation.getX();
		goalY = translation.getY();

		xOnlyMode = Math.abs(goalX) > Math.abs(goalY);

		xMovePID = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);

		yMovePID = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);

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

		if (Math.abs(x - goalX) < RobotMovementConstants.POSITION_TOLERANCE_METERS
				&& Math.abs(y - goalY) < RobotMovementConstants.POSITION_TOLERANCE_METERS)
			atSetpointCounter += TimedRobot.kDefaultPeriod;
		else
			atSetpointCounter = 0;

		double xSpeed = xMovePID.calculate(x, goalX);
		double ySpeed = yMovePID.calculate(y, goalY);

		if (xOnlyMode)
			ySpeed = 0;
		else
			xSpeed = 0;

		final ChassisSpeeds speeds = new ChassisSpeeds(
				xSpeed,
				ySpeed,
				0);

		drivetrain.setDesiredState(speeds);
	}

	@Override
	public boolean isFinished() {
		return atSetpointCounter > RobotMovementConstants.AT_SETPOINT_TOLERANCE_TIME_SECONDS;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
