package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotMovementConstants;

/** Base Command for child commands that need to automatically drive to a certain Pose on the field */
public class DriveToPoseBase extends Command {
	private final SwerveDrivetrain drivetrain;
	private final PIDController xController, yController, rotationController;

	/**
	 * Create a new DriveToPose command. Tries to drive to a set Pose based on odometry.
	 * 
     * <p>This drives relative to the robot starting position,
	 * so a pose of +2x and +1y will drive to the position 2 meters forward and 1 meter left of whether the robot started,
     * where forward is whatever direction the robot started in</p>
     * 
     * <p>The last place the drivetrain position was reset counts as the starting position</p>
	 * 
	 * @param drivetrain the drivetrain of the robot
	 */
	public DriveToPoseBase(SwerveDrivetrain drivetrain) {

		// Save drivetrain
		this.drivetrain = drivetrain;

		// Setup all PID controllers
		xController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		xController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);

		yController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);

		rotationController = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotationController.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);

		// Add drivetrain as a requirement so no other commands try to use it
		addRequirements(this.drivetrain);
	}

	/**
	 * 
	 * 
	 * @param targetPose Pose that robot will drive to once command is scheduled
	 */
	public void setDesiredPosition(Pose2d targetPose) {
		xController.setSetpoint(targetPose.getX());
		yController.setSetpoint(targetPose.getY());
		rotationController.setSetpoint(targetPose.getRotation().getRadians());
	}

	public Pose2d getPosition() {
		return drivetrain.getPosition();
	}

	/** Put all swerve modules in default positions */
	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {

		// Get our current pose
		final Pose2d measuredPosition = getPosition();

		// Put current pose position on SmartDashboard
		SmartDashboard.putNumber("PoseY", measuredPosition.getY());
		SmartDashboard.putNumber("PoseX", measuredPosition.getX());
		SmartDashboard.putNumber("PoseDegrees", measuredPosition.getRotation().getDegrees());

		// Calculate our robot speeds with the PID controllers
		final ChassisSpeeds speeds = new ChassisSpeeds(
				xController.calculate(measuredPosition.getX()),
				yController.calculate(measuredPosition.getY()),
				rotationController.calculate(measuredPosition.getRotation().getRadians()));

		// Set those speeds
		drivetrain.setDesiredState(speeds);
	}

	/** Finish once all controllers are within tolerance */
	@Override
	public boolean isFinished() {
		return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
	}

	/** Stop all swerve modules at end */
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
