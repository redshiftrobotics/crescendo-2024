package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Vision subsystem */
public class Vision extends SubsystemBase {

	private final static boolean DEBUG_INFO = false;

	final PhotonCamera camera;
	final Transform3d robotToCamera;
	final Transform3d cameraToRobot;

	/**
	 * Create new PhotonCamera subsystem
	 * 
	 * @param cameraName    name of PhotonCamera
	 * @param robotToCamera distance from the camera to the front center point
	 *                      of the robot
	 */
	public Vision(String cameraName, Transform3d robotToCamera) {
		camera = new PhotonCamera(cameraName);
		this.robotToCamera = robotToCamera;
		cameraToRobot = robotToCamera.inverse();
	}

	// If you need the direction (yaw) from the robot to the tag, use something like
	// this:
	// Transform3d distToTag = vision.getDistToTag(tagID);
	// Rotation2d yawToTag = new Rotation2d(distToTag.getX(), distToTag.getY());

	/**
	 * Get distance to the best target found by the camera
	 * 
	 * @return the position of the tag (translation and rotation) based on the
	 *         center of the robot. Returns null if no target found.
	 * 
	 */
	public Transform3d getDistToTag() {
		PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
		if (target == null) {
			return null;
		}
		return target.getBestCameraToTarget();
	}

	/**
	 * Get distance to the desired tag
	 * 
	 * @param tagID the fiducial ID of the desired tag
	 * @return the position of the tag (translation and rotation) based on the
	 *         center of the robot. Returns null if no target found
	 */

	public Transform3d getDistToTag(int tagID) {
		for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
			if (target.getFiducialId() == tagID)
				return target.getBestCameraToTarget().plus(cameraToRobot);
		}
		return null;
	}

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
