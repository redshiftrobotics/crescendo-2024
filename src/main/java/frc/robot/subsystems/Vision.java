package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// How to make Subsystem (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

/**
 * 
 */

public class Vision extends SubsystemBase {
    /** Constructor. Creates a new ExampleSubsystem. */
    PhotonCamera camera;
    
    
    PhotonTrackedTarget bestTag = null;
    List<PhotonTrackedTarget> targets = null;
    
    public Vision(String cameraName) {
        camera = new PhotonCamera(cameraName);

    }

    /**
     * Gives Transform3d from the robot center to the desired target
     * @return
     */
    public Transform3d getDistToTag() {
        return bestTag.getBestCameraToTarget();
    }

    /**
     * Gives Transform3d from robot center to the desired target
     * @param tagID The fiducial ID of the desired April Tag
     * @return Returns null if the tag cannot be found
     * 
     */
    public Transform3d getDistToTag(int tagID) {
        Transform3d dist = null;
        for(int i = 0; i < targets.size(); i++)
        {
            if (targets.get(i).getFiducialId() == tagID) {
                dist = targets.get(i).getBestCameraToTarget();
                break;
            }
        }
        
        return dist;
    }
    /**
     * This method is called periodically by the CommandScheduler, about every 20ms.
     * It should be used for updating subsystem-specific state that you don't want to offload to a Command.
     * Try to avoid "doing to much" in this method (for example no driver control here).
     */
    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        // This method will be called once per scheduler run
        bestTag = result.getBestTarget();
        targets = result.getTargets();
        if(bestTag != null) {
            SmartDashboard.putNumber("Tag ID", bestTag.getFiducialId());
            Transform3d tagPose = bestTag.getBestCameraToTarget();
            Rotation3d tagRot = tagPose.getRotation();
            // SmartDashboard.putNumberArray("Tag X Y Z", new double[] {tagPose.getX(), tagPose.getY(), tagPose.getZ()});
            // SmartDashboard.putNumberArray("Tag Yaw, Pitch, Roll", new double[] {
            //     Units.radiansToDegrees(tagRot.getZ()),
            //     Units.radiansToDegrees(tagRot.getY()),
            //     Units.radiansToDegrees(tagRot.getX())});
            SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
            SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
            SmartDashboard.putNumber("Tag Pose Z", tagPose.getZ());
            SmartDashboard.putNumber("Tag Pose Yaw", tagRot.getZ());
            SmartDashboard.putNumber("Tag Pose Pitch", tagRot.getY());
            SmartDashboard.putNumber("Tag Pose Roll", tagRot.getX());
            SmartDashboard.putNumber("Tag Yaw", bestTag.getYaw());
            SmartDashboard.putNumber("Tag Pitch", bestTag.getPitch());
        } else {
            SmartDashboard.putNumber("Tag Pose ID", -1);
            SmartDashboard.putNumber("Tag Pose X", -1);
            SmartDashboard.putNumber("Tag Pose Y", -1);
            SmartDashboard.putNumber("Tag Pose Z", -1);
            SmartDashboard.putNumber("Tag Pose Yaw", -1);
            SmartDashboard.putNumber("Tag Pose Pitch", -1);
            SmartDashboard.putNumber("Tag Pose Roll", -1);
            SmartDashboard.putNumber("Tag Yaw", 0);
            SmartDashboard.putNumber("Tag Pitch", 0);
            // SmartDashboard.putNumberArray("Tag X Y Z", new double[] {0, 0, 0});
            // SmartDashboard.putNumberArray("Tag Yaw, Pitch, Roll", new double[] {0, 0, 0});
        }

        
    }
}
