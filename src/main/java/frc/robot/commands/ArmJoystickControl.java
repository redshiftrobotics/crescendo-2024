package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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


        lowerArmButton = new OptionButton(joystick, 11, ActivationMode.HOLD);
        raiseArmButton = new OptionButton(joystick, 12, ActivationMode.HOLD);

        joystick.povUp().onTrue(Commands.run(arm::setArmToSpeakerPosition));
        joystick.povRight().onTrue(Commands.run(arm::setArmToAmpPosition));
        joystick.povDown().onTrue(Commands.run(arm::setArmToIntakePosition));

        addRequirements(arm);
    }


    /** 
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {


        //two buttons determining the raising and lowering of the arm
        arm.changeArmAngleDegreesBy(Double.valueOf(raiseArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod * ArmConstants.DEGREES_PER_SECOND);
        arm.changeArmAngleDegreesBy(Double.valueOf(-lowerArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod * ArmConstants.DEGREES_PER_SECOND);
    }
}
