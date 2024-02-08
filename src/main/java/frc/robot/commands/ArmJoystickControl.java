package frc.robot.commands;

import frc.robot.subsystems.Arm;

import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class ArmJoystickControl extends Command {
    private final Arm arm;
    private final CommandJoystick joystick;

    private final OptionButton raiseArmButton;
    private final OptionButton lowerArmButton;

    private double armPos;

    public ArmJoystickControl(Arm arm, CommandJoystick joystick) {
        this.arm = arm;
        this.joystick = joystick;


        lowerArmButton = new OptionButton(joystick, 101, ActivationMode.HOLD);
        raiseArmButton = new OptionButton(joystick, 202, ActivationMode.HOLD);

        addRequirements(arm);
    }


    /** 
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {

        armPos += raiseArmButton.getStateAsInt();
        armPos -= lowerArmButton.getStateAsInt();
        

        arm.setArmAngleDegrees(armPos);
    }
}
