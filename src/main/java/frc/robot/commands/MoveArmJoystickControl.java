package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class MoveArmJoystickControl extends Command {

    public MoveArmJoystickControl(Arm arm, CommandJoystick joystick) {
        

    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {
        double armSpeed = 0;

        
    }
}