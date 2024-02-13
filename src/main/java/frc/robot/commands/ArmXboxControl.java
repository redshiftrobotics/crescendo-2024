package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmXboxControl extends Command {
    private final Arm arm;
    private final CommandXboxController xbox;

    private final OptionButton raiseArmButton;
    private final OptionButton lowerArmButton;


    public ArmXboxControl(Arm arm, CommandXboxController xbox ) {
        this.arm = arm;
        this.xbox = xbox;


        lowerArmButton = new OptionButton(xbox, 5, ActivationMode.HOLD);
        raiseArmButton = new OptionButton(xbox, 6, ActivationMode.HOLD);

        xbox.povUp().onTrue(Commands.run(arm::setArmToSpeakerPosition));
        xbox.povRight().onTrue(Commands.run(arm::setArmToAmpPosition));
        xbox.povDown().onTrue(Commands.run(arm::setArmToIntakePosition));

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
