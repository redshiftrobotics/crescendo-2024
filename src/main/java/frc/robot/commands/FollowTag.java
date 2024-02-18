package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically drive a follow a tag a certain translation away */
public class FollowTag extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Vision vision;
	private final Integer tagID; // Integer as opposed to int so it can be null for best tag
	private final Transform2d targetDistance;

	private final Double loseTagAfterSeconds; // Double as opposed to double so it can be null for never lose mode.
	private double secondsSinceTagLastSeen;

	/**
	 * Create a new FollowTag command. Tries to follow a tag while staying a certain
	 * distance away.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the the tag to follow, null
	 *                            for best tag
	 * @param targetDistanceToTag the target distance away from the tag to be
	 * @param loseTagAfterSeconds how long to wait before giving up on rediscover
	 *                            tag, set to null to never finish
	 */
	public FollowTag(SwerveDrivetrain drivetrain, Vision vision, Transform2d targetDistanceToTag, Integer tagID,
			Double loseTagAfterSeconds) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;

		this.targetDistance = targetDistanceToTag;
		this.loseTagAfterSeconds = loseTagAfterSeconds;

		addRequirements(drivetrain);
	}

	/**
	 * Create a new FollowTag command. Tries to follow a tag while staying a certain
	 * distance away.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the the tag to follow, null
	 *                            for whatever best is
	 * @param targetDistanceToTag the target distance away from the tag to be
	 * @param loseTagAfterSeconds how long to wait before giving up on rediscover
	 *                            tag, set to null to never finish
	 */
	public FollowTag(SwerveDrivetrain drivetrain, Vision vision, Translation2d targetDistanceToTag, Integer tagID,
			Double loseTagAfterSeconds) {
		this(drivetrain, vision, new Transform2d(targetDistanceToTag, new Rotation2d()), tagID, loseTagAfterSeconds);
	}

	@Override
	public void initialize() {
		secondsSinceTagLastSeen = 0;
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {

		final Transform3d tagPosition3d = (tagID == null) ? vision.getDistToTag() : vision.getDistToTag(tagID);

		if (tagPosition3d == null) {
			secondsSinceTagLastSeen += TimedRobot.kDefaultPeriod;
			drivetrain.stop();
		} else {

			secondsSinceTagLastSeen = 0;

			// https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
			// https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#getting-apriltag-data-from-a-target
			// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation3d.html#getZ()
			final Transform2d tagPosition = new Transform2d(
					tagPosition3d.getX(),
					tagPosition3d.getY(),
					new Rotation2d(tagPosition3d.getX(), tagPosition3d.getY()));

			final Transform2d driveTransform = tagPosition.plus(
					targetDistance.inverse().plus(new Transform2d(new Translation2d(), tagPosition.getRotation())));

			drivetrain.setDesiredPosition(drivetrain.getPosition().plus(driveTransform));
		}

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return (loseTagAfterSeconds != null) && (secondsSinceTagLastSeen < loseTagAfterSeconds);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.clearDesiredPosition();
		drivetrain.stop();
	}
}
