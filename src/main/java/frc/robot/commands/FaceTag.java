package frc.robot.commands;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class FaceTag extends Command {
    private final Vision vision;
    private final SwerveDrivetrain drivetrain;
    private final PIDController rotationPID;

    public FaceTag(Vision vision, SwerveDrivetrain drivetrain, Map<Integer,Double> targetTagPos){
        this.vision = vision;
        this.drivetrain = drivetrain;
        rotationPID = new PIDController(
            RobotMovementConstants.ROTATION_PID_P,
            RobotMovementConstants.ROTATION_PID_I,
            RobotMovementConstants.ROTATION_PID_D);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.toDefaultStates();
    }

    @Override
    public void execute(){   
    }

}
