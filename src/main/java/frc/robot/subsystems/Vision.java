package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// How to make Subsystem (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

/** Vision subsystem */
public class Vision extends SubsystemBase {

	private final static boolean DEBUG_INFO = true;

	final PhotonCamera camera;
	final Transform3d cameraToFrontCenter;

	/**
	 * Create new PhotonCamera subsystem
	 * 
	 * @param cameraName          name of PhotonCamera
	 * @param cameraToFrontCenter distance from the camera to the front center point
	 *                            of the robot
	 */
	public Vision(String cameraName, Transform3d cameraToFrontCenter) {
		camera = new PhotonCamera(cameraName);
		this.cameraToFrontCenter = cameraToFrontCenter;
	}

	/**
	 * Get best april tag target
	 * 
	 * @return Object of best target
	 */
	public PhotonTrackedTarget getTag() {
		return camera
				.getLatestResult()
				.getBestTarget();
	}

	/**
	 * Gives Transform3d from robot center to the desired target
	 * 
	 * @param tagID The fiducial ID of the desired April Tag
	 * @return returns first tag with matching ID, null if None are found
	 */
	public PhotonTrackedTarget getTag(int tagID) {
		for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
			if (target.getFiducialId() == tagID)
				return target;
		}
		return null;
	}

	public Transform3d getDistanceToTarget(PhotonTrackedTarget tag) {
		return tag
				.getBestCameraToTarget()
				.plus(cameraToFrontCenter);
	}

	/**
	 * This method is called periodically by the CommandScheduler, about every 20ms.
	 * It should be used for updating subsystem-specific state that you don't want
	 * to offload to a Command.
	 * Try to avoid "doing to much" in this method (for example no driver control
	 * here).
	 */
	@Override
	public void periodic() {
		if (!DEBUG_INFO)
			return;

		PhotonPipelineResult result = camera.getLatestResult();

		PhotonTrackedTarget bestTag = result.getBestTarget();

		if (bestTag == null) {
			bestTag = new PhotonTrackedTarget(-1, -1, -1, -1, -1,
					new Transform3d(new Translation3d(-1, -1, -1), new Rotation3d(-1, -1, -1)), null, 0,
					new ArrayList<>(), new ArrayList<>());
		}

		SmartDashboard.putNumber("Tag ID", bestTag.getFiducialId());
		SmartDashboard.putNumber("Tag Yaw", bestTag.getYaw());
		SmartDashboard.putNumber("Tag Pitch", bestTag.getPitch());
		SmartDashboard.putNumber("Tag Skew", bestTag.getSkew());

		Transform3d tagPose = bestTag.getBestCameraToTarget();

		SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
		SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
		SmartDashboard.putNumber("Tag Pose Z", tagPose.getZ());

		Rotation3d tagPoseRotation = tagPose.getRotation();

		SmartDashboard.putNumber("Tag Pose Yaw", tagPoseRotation.getZ());
		SmartDashboard.putNumber("Tag Pose Pitch", tagPoseRotation.getY());
		SmartDashboard.putNumber("Tag Pose Roll", tagPoseRotation.getX());
	}
}
