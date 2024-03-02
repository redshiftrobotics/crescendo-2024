package frc.robot.commands;

import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically drive a follow a tag a certain translation away */
public class FollowTag extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Vision vision;
	private final int tagID;

	private final PIDController xController, yController, rotationController;

	/**
	 * Create a new FollowTag command. Tries to follow a tag while staying a certain
	 * distance away.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the the tag to follow, -1 for best tag
	 * @param targetDistanceToTag the target distance away from the tag to be in meters
	 * @param loseTagAfterSeconds how long to wait before giving up on rediscover
	 *                            tag, set to null to never finish
	 */
	public FollowTag(SwerveDrivetrain drivetrain, Vision vision, Transform2d targetDistanceToTag, int tagID) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;

		xController = new PIDController(RobotMovementConstants.TRANSLATION_PID_P, RobotMovementConstants.TRANSLATION_PID_I, RobotMovementConstants.TRANSLATION_PID_D); xController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		xController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		xController.setSetpoint(targetDistanceToTag.getX());
		
		yController = new PIDController(RobotMovementConstants.TRANSLATION_PID_P, RobotMovementConstants.TRANSLATION_PID_I, RobotMovementConstants.TRANSLATION_PID_D);
		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		xController.setSetpoint(targetDistanceToTag.getY());
		
		rotationController = new PIDController(RobotMovementConstants.ROTATION_PID_P, RobotMovementConstants.ROTATION_PID_I, RobotMovementConstants.ROTATION_PID_D);
		rotationController.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		xController.setSetpoint(targetDistanceToTag.getRotation().getRadians());

		addRequirements(drivetrain);
	}

	public FollowTag(SwerveDrivetrain drivetrain, Vision vision, Translation2d targetDistanceToTag, int tagID) {
		this(drivetrain, vision, new Transform2d(targetDistanceToTag, new Rotation2d()), tagID);
	}
	
	public FollowTag(SwerveDrivetrain drivetrain, Vision vision, double targetDistanceToTag, int tagID) {
		this(drivetrain, vision, new Transform2d(new Translation2d(targetDistanceToTag, 0), new Rotation2d()), tagID);
	}

	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {
		double forwardSpeed = 0;
		double rotationSpeed = 0;
		double leftSpeed = 0;

		Transform3d transform = vision.getTransformToTag(tagID);

		if (transform != null){
			double forward = transform.getX();
			double left = transform.getY();
			Rotation2d rotation = new Rotation2d(forward, left);

			forwardSpeed = -xController.calculate(forward);
			leftSpeed = yController.calculate(left);
			rotationSpeed = -rotationController.calculate(rotation.getRadians());
		}

    	drivetrain.setDesiredState(new ChassisSpeeds(forwardSpeed, leftSpeed, rotationSpeed));
		drivetrain.updateSmartDashboard();
  	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}