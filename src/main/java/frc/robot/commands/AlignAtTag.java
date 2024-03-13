package frc.robot.commands;

import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically align at a tag, ends once facing the tag */
public class AlignAtTag extends Command {
	private final SwerveDrivetrain drivetrain;
	private final ChassisDriveInputs chassisDriveInputs;

	private final Vision vision;
	private final int tagID;

	private final PIDController yController;

	/**
	 * Create a new AlignAtTag command. Tries to constants Align at a tag while still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the tag to turn to, -1 for best tag
	 */
	public AlignAtTag(SwerveDrivetrain drivetrain, Vision vision, int tagID, ChassisDriveInputs chassisDriveInputs) {
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
	public AlignAtTag(SwerveDrivetrain drivetrain, Vision vision, int tagID) {
		this(drivetrain, vision, tagID, null);
	}

	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {
		Transform3d transform = vision.getTransformToTag(tagID);

		double ySpeed = 0;
		if (transform != null) {
			ySpeed = yController.calculate(transform.getY());
		}

		double xSpeed = 0;
		double rotationSpeed = 0;
		boolean fieldRelative = false;
		if (chassisDriveInputs != null) {
			xSpeed = chassisDriveInputs.getX();
			// rotationSpeed = chassisDriveInputs.getRotation()
			fieldRelative = chassisDriveInputs.isFieldRelative();
		}

		ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, 0, rotationSpeed);
		if (fieldRelative) desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, drivetrain.getHeading());
		desiredSpeeds.vyMetersPerSecond = ySpeed;

		drivetrain.setDesiredState(desiredSpeeds, false);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return yController.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}