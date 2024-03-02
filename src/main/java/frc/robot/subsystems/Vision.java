package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Vision subsystem */
public class Vision extends SubsystemBase {

	private final static boolean DEBUG_INFO = false;

	public static enum Tags {
		/**
		 * Get the best target in the current pipeline, best is determined by whatever
		 * the target sort mode is.
		 */
		BEST_TARGET(-1),

		BLUE_SOURCE_RIGHT(1);

		public final int id;

		Tags(int id) {
			this.id = id;
		}
	}

	private final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField
			.loadAprilTagLayoutField();

	private final PhotonCamera camera;
	private final Transform3d cameraToRobot;

	private final PhotonPoseEstimator photonPoseEstimator;

	/**
	 * Create new PhotonCamera subsystem
	 * 
	 * @param cameraName    name of PhotonCamera
	 * @param robotToCamera distance from the camera to the center of the robot
	 */
	public Vision(String cameraName, Transform3d robotToCamera) {
		camera = new PhotonCamera(cameraName);

		cameraToRobot = robotToCamera.inverse();

		// MAKE SURE “Do Multi-Target Estimation” IS ACTIVATED ON OUTPUT TAB OF
		// DASHBOARD
		// Current hostname of a device is http://photonvision.local:5800/#/dashboard,
		// make sure on local (robot) network
		// Docs:
		// https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html
		photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				robotToCamera);
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return photonPoseEstimator.update();
	}

	public PhotonTrackedTarget getTag() {
		return getTag(Tags.BEST_TARGET.id);
	}

	public PhotonTrackedTarget getTag(int tagID) {
		PhotonPipelineResult result = camera.getLatestResult();

		if (tagID == Tags.BEST_TARGET.id)
			return result.getBestTarget();

		for (PhotonTrackedTarget target : result.getTargets()) {
			if (target.getFiducialId() == tagID)
				return target;
		}

		return null;
	}

	/**
	 * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
	 * object/fiducial tag space (X forward, Y left, Z up) with the lowest
	 * reprojection error
	 * 
	 * @return The position of the tag (translation and rotation) based on the
	 *         center of the robot. Returns null if no tag found.
	 * 
	 */
	public Transform3d getTransformToTag() {
		return getTransformToTag(Tags.BEST_TARGET.id);
	}

	/**
	 * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
	 * object/fiducial tag space (X forward, Y left, Z up) with the lowest reprojection error
	 * 
	 * @param tagID the fiducial ID of the desired tag, -1 for best tag
	 * @return the position of the tag (translation and rotation) based on the
	 *         center of the robot. Returns null if no tag found
	 */
	public Transform3d getTransformToTag(int tagID) {
		PhotonTrackedTarget tag = getTag(tagID);
		if (tag == null)
			return null;
		return tag.getBestCameraToTarget();
	}

	public Pose3d getFieldRelativePoseWithTag(int tagID) {

		PhotonTrackedTarget tag = getTag(tagID);

		Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tag.getFiducialId());

		if (tagPose == null)
			return null;

		return PhotonUtils.estimateFieldToRobotAprilTag(
				tag.getBestCameraToTarget(),
				tagPose.get(),
				cameraToRobot);
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