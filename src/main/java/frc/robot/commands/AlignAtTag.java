package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Command to automatically align at a tag, ends once facing the tag */
public class AlignAtTag extends Command {
	private final SwerveDrivetrain drivetrain;
	private final ChassisDriveInputs chassisDriveInputs;

	private final Vision vision;
	private final int[] tagID;

	private final PIDController yController, rotatePID;

	/**
	 * Create a new AlignAtTag command. Tries to constants Align at a tag while
	 * still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to turn to, -1 for best tag
	 */
	public AlignAtTag(SwerveDrivetrain drivetrain, Vision vision, int[] tagID, ChassisDriveInputs chassisDriveInputs,
			Rotation2d rotation) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;
		this.chassisDriveInputs = chassisDriveInputs;

		yController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);

		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		yController.setSetpoint(0);

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotatePID.setSetpoint(rotation.getRadians());

		addRequirements(drivetrain, chassisDriveInputs);
	}

	/**
	 * Create a new AlignAtTag command. Tries to Align at a tag.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to turn to, null for best
	 *                   tag
	 */
	public AlignAtTag(SwerveDrivetrain drivetrain, Vision vision, int[] tagID, Rotation2d rotation) {
		this(drivetrain, vision, tagID, null, rotation);
	}

	@Override
	public void initialize() {
		rotatePID.reset();
		yController.reset();
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {
		Transform3d transform = vision.getTransformToTag(tagID[0]);
		if (transform == null) {
			transform = vision.getTransformToTag(tagID[1]);
		}

		if (transform != null) {
			SmartDashboard.putNumber("X", transform.getX());
			SmartDashboard.putNumber("Y", transform.getY());
		} else {
			SmartDashboard.putNumber("X", -1);
			SmartDashboard.putNumber("Y", -1);
		}

		double ySpeed = 0;
		double rotationSpeed = 0;
		if (transform != null) {
			ySpeed = yController.calculate(transform.getY());
			rotationSpeed = rotatePID.calculate(drivetrain.getHeading().getRadians());
		}

		double xSpeed = 0;
		boolean fieldRelative = false;
		if (chassisDriveInputs != null) {
			xSpeed = chassisDriveInputs.getX();
			fieldRelative = chassisDriveInputs.isFieldRelative();
		}

		ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, 0, rotationSpeed);
		if (fieldRelative)
			desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, drivetrain.getHeading());
		desiredSpeeds.vyMetersPerSecond = ySpeed;

		drivetrain.setDesiredState(desiredSpeeds, false);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return yController.atSetpoint() && rotatePID.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}