package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

/**
 * This is the default command for the drivetrain, allowing for remote operation with PS4 controller
 */
public class SwerveDrivePS4Control extends SwerveDriveBaseControl {
    private final OptionButton preciseModeButton;
    private final OptionButton boostModeButton;

    /**
     * Creates a new SwerveDrivePS4Control Command
     * 
     * @param drivetrain The drive train of the robot
     * @param driverPS4Controller The PS4 controller used to control the drive train
     */
    public SwerveDrivePS4Control(SwerveDrivetrain drivetrain, CommandPS4Controller driverPS4Controller){
        super(drivetrain, driverPS4Controller);

        // Create and configure buttons
        preciseModeButton = new OptionButton(driverPS4Controller::circle, ActivationMode.TOGGLE);
        boostModeButton = new OptionButton(driverPS4Controller::triangle, ActivationMode.TOGGLE);

        //Tell the command scheduler we are using the drivetrain.
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        final CommandPS4Controller ps4Controller = (CommandPS4Controller) controller;

        final double leftX = ps4Controller.getLeftX();
        final double leftY = ps4Controller.getLeftY();
        final double rightX = ps4Controller.getRightX();

        final int speedLevel = 1
            - preciseModeButton.getStateAsInt()
            + boostModeButton.getStateAsInt();

        final ChassisSpeeds speeds = new ChassisSpeeds(
            leftX * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
            leftY * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
            rightX * DriverConstants.maxSpeedOptionsRotation[speedLevel]
        );

        // Display relevant data on shuffleboard.
        SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);        
        SmartDashboard.putBoolean("Field Relieve", false);

        drivetrain.setDesiredState(speeds,  true, false);
    }
}
