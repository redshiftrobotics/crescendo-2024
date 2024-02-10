package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/** Command to automatically drive a follow a tag a certain translation away */
public class FollowTag extends DriveToPoseBase {
	private final int tagID;
	private final Transform2d targetDistance;

	private final double loseTagAfterSeconds;
	private double secondsSinceTagLastSeen;

	/**
	 * Create a new FollowTag command. Tries to follow a tag while staying a certain distance away.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param tagID               the numerical ID of the the tag to follow
	 * @param targetDistanceToTag the target distance away from the tag to be
	 * @param loseTagAfterSeconds how long to wait before giving up on rediscover
	 *                            tag, set to -1 to never finish
	 */
	public FollowTag(SwerveDrivetrain drivetrain, int tagID, Transform2d targetDistanceToTag, int loseTagAfterSeconds) {
		super(drivetrain);

		this.tagID = tagID;
		this.targetDistance = targetDistanceToTag;
		this.loseTagAfterSeconds = loseTagAfterSeconds;
	}

	/**
	 * Create a new FollowTag command. Tries to follow a tag while staying a certain distance away.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param tagID               the numerical ID of the the tag to follow
	 * @param targetDistanceToTag the target distance away from the tag to be
	 * @param loseTagAfterSeconds how long to wait before giving up on rediscover
	 *                            tag, set to -1 to never finish
	 */
	public FollowTag(SwerveDrivetrain drivetrain, int tagID, Translation2d targetDistanceToTag, int loseTagAfterSeconds) {
		this(drivetrain, tagID, new Transform2d(targetDistanceToTag, new Rotation2d()), loseTagAfterSeconds);
	}

	@Override
	public void execute() {

		// Sudo code, assume distance from front center of robot
		final Transform3d tagPosition3d = new Transform3d();

		if (tagPosition3d != null) {
			// https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
			// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation3d.html#getZ()
			final Transform2d tagPosition = new Transform2d(
					tagPosition3d.getZ(),
					tagPosition3d.getX(),
					Rotation2d.fromRadians(-tagPosition3d.getRotation().getZ()));
	
			final Transform2d driveTransform = tagPosition.plus(targetDistance.inverse());
	
			setDesiredPosition(getPosition().plus(driveTransform));
		}
		else {
			stop();
		}

		super.execute();
	}

	@Override
	public boolean isFinished() {
		return (loseTagAfterSeconds != -1) && (secondsSinceTagLastSeen < loseTagAfterSeconds);
	}
}
