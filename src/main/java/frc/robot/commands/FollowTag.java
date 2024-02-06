package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.RobotMovementConstants;

public class FollowTag extends Command {
	private final SwerveDrivetrain drivetrain;
	private final int tagID;
	private final Translation2d targetDistance;
	private final PIDController xController, yController, rotationController;

	public FollowTag(SwerveDrivetrain drivetrain, int tagID, Translation2d targetDistanceToTag) {
		this.drivetrain = drivetrain;
		this.tagID = tagID;
		this.targetDistance = targetDistanceToTag;

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
	}

	@Override
	public void execute() {

	}
}
