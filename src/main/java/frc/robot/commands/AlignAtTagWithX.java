package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Command to automatically align at a tag, ends once facing the tag */
public class AlignAtTagWithX extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Vision vision;
	private final int[] tagID;

	private boolean done = false;

	private final PIDController xController, yController, rotatePID;

	/**
	 * Create a new AlignAtTag command. Tries to constants Align at a tag while
	 * still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to turn to, -1 for best tag
	 */
	public AlignAtTagWithX(SwerveDrivetrain drivetrain, Vision vision, int[] tagID,
			Rotation2d rotation, double xDistance) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;

		yController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		yController.setSetpoint(0);
		
		xController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		xController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		xController.setSetpoint(xDistance);

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotatePID.setSetpoint(rotation.getRadians());

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		rotatePID.reset();
		yController.reset();
		drivetrain.toDefaultStates();

		done = (vision.getTransformToTag(tagID[0]) == null) && (vision.getTransformToTag(tagID[1]) == null);
	}

	@Override
	public void execute() {
		Transform3d transform = vision.getTransformToTag(tagID[0]);
		if (transform == null) {
			transform = vision.getTransformToTag(tagID[1]);
		}

		double xSpeed = 0;
		double ySpeed = 0;
		double rotationSpeed = 0;
		if (transform != null) {
			xSpeed = yController.calculate(transform.getX());
			ySpeed = yController.calculate(transform.getY());
			rotationSpeed = rotatePID.calculate(drivetrain.getHeading().getRadians());
		}

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), false);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return done || (yController.atSetpoint() && rotatePID.atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}