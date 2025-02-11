package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Vision subsystem */
public class Vision extends SubsystemBase {

	// private final static boolean DEBUG_INFO = false;

	private boolean visionEnabled = false;

	// final PhotonCamera camera;
	final Transform3d robotToCamera;
	final Transform3d cameraToRobot;

	/**
	 * Create new PhotonCamera subsystem
	 * 
	 * @param cameraName    name of PhotonCamera
	 * @param robotToCamera distance from the camera to the center of the robot
	 */
	public Vision(String cameraName, Transform3d robotToCamera) {
		// camera = new PhotonCamera(cameraName);
		this.robotToCamera = robotToCamera;
		cameraToRobot = robotToCamera.inverse();

		SmartDashboard.putBoolean("Use Vision", visionEnabled);
	}

	public boolean isEnabled() {
		return visionEnabled;
	}

	public void enableUsing() {
		visionEnabled = true;
	}

	public void disableUsing() {
		visionEnabled = false;
	}

	public void toggleUsing() {
		System.out.println("Run");
		visionEnabled = !visionEnabled;
	}

	/**
	 * Get distance to the distance to the best tag found by the camera
	 * 
	 * @return The position of the tag (translation and rotation) based on the
	 *         center of the robot. Returns null if no tag found.
	 * 
	 */
	public Transform3d getTransformToTag() {
		// PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
		// if (target == null) {
		// 	return null;
		// }
		// return target.getBestCameraToTarget().plus(cameraToRobot);
		return null;
	}

	/**
	 * Get distance to the desired tag
	 * 
	 * @param tagID the fiducial ID of the desired tag
	 * @return the position of the tag (translation and rotation) based on the
	 *         center of the robot. Returns null if no tag found
	 */
	public Transform3d getTransformToTag(int tagID) {
		if (tagID == -1)
			return getTransformToTag();

		// var results = camera.getLatestResult();

		// for (PhotonTrackedTarget target : results.getTargets()) {
		// 	if (target.getFiducialId() == tagID)
		// 		return target.getBestCameraToTarget().plus(cameraToRobot);
		// }
		return null;
	}

	@Override
	@SuppressWarnings("unused")
	public void periodic() {

		SmartDashboard.putBoolean("Use Vision", visionEnabled);

		// if (visionEnabled && DEBUG_INFO) {
		// 	PhotonPipelineResult result = camera.getLatestResult();
		// 	PhotonTrackedTarget bestTag = result.getBestTarget();

		// 	if (bestTag == null) {
		// 		bestTag = new PhotonTrackedTarget(-1, -1, -1, -1, -1,
		// 				new Transform3d(new Translation3d(-1, -1, -1), new Rotation3d(-1, -1, -1)), null, 0,
		// 				new ArrayList<>(), new ArrayList<>());
		// 	}

		// 	SmartDashboard.putNumber("Tag ID", bestTag.getFiducialId());
		// 	SmartDashboard.putNumber("Tag Yaw", bestTag.getYaw());
		// 	SmartDashboard.putNumber("Tag Pitch", bestTag.getPitch());
		// 	SmartDashboard.putNumber("Tag Skew", bestTag.getSkew());

		// 	Transform3d tagPose = bestTag.getBestCameraToTarget();

		// 	SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
		// 	SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
		// 	SmartDashboard.putNumber("Tag Pose Z", tagPose.getZ());

		// 	Rotation3d tagPoseRotation = tagPose.getRotation();

		// 	SmartDashboard.putNumber("Tag Pose Yaw", tagPoseRotation.getZ());
		// 	SmartDashboard.putNumber("Tag Pose Pitch", tagPoseRotation.getY());
		// 	SmartDashboard.putNumber("Tag Pose Roll", tagPoseRotation.getX());
		// }
	}
}