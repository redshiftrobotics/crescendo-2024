package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// How to make Subsystem (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

/**
 * How to make a Subsystem:
 * 1. Copy this file, remember that class name has to match 
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
        } else {
            SmartDashboard.putNumber("Tag ID", -1);
        }

        
    }
}
