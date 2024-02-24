package frc.robot.commands;

import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically drive a follow a tag a certain translation away */
public class TurnToTag extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Vision vision;
	private final Integer tagID;  // Integer as opposed to int so it can be null for best tag

	private final PIDController rotatePID;

	/**
	 * Create a new FollowTag command. Tries to follow a tag while staying a certain
	 * distance away.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the the tag to turn to, null for best tag
	 */
	public TurnToTag(SwerveDrivetrain drivetrain, Vision vision, Integer tagID) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;

        rotatePID = new PIDController(
            RobotMovementConstants.ROTATION_PID_P,
            RobotMovementConstants.ROTATION_PID_I,
            RobotMovementConstants.ROTATION_PID_D);
        rotatePID.enableContinuousInput(-Math.PI, Math.PI);
        rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		drivetrain.toDefaultStates();

        final PhotonTrackedTarget tag = (tagID == null) ? vision.getTag() : vision.getTag(tagID);
        rotatePID.setSetpoint(tag.getYaw());
	}


	@Override
	public void execute() {
    	final double currentAngle = drivetrain.getHeading().getRadians();

		double turnsSeed = rotatePID.calculate(currentAngle);
		SmartDashboard.putNumber("Turn Speed Auto", turnsSeed);

		drivetrain.setDesiredState(new ChassisSpeeds(0, 0, turnsSeed));

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return rotatePID.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
