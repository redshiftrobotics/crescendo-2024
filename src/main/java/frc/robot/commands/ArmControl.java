package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.OptionButton;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class ArmControl extends Command {
	private final Arm arm;

	private final OptionButton raiseArmButton;
	private final OptionButton lowerArmButton;

    private final OptionButton speakerPositionButton;
    private final OptionButton ampPositionButton;
    private final OptionButton intakePositionButton;


	public ArmControl(Arm arm, OptionButton raiseArmButton, OptionButton lowerArmButton,
            OptionButton speakerPositionButton, OptionButton ampPositionButton, OptionButton intakePositionButton) {
        this.arm = arm;

        this.raiseArmButton = raiseArmButton;
        this.lowerArmButton = lowerArmButton;

        this.speakerPositionButton = speakerPositionButton;
        this.ampPositionButton = ampPositionButton;
        this.intakePositionButton = intakePositionButton;


		addRequirements(arm);
	}


	
	@Override
	public void initialize() {
	}

	/**
	 * execute() is called repeatedly while a command is scheduled, about every
	 * 20ms.
	 */
	@Override
	public void execute() {

        // Arm Motor
		arm.changeArmAngleDegreesBy(Double.valueOf(raiseArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod * ArmConstants.DEGREES_PER_SECOND);
        arm.changeArmAngleDegreesBy(Double.valueOf(-lowerArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod * ArmConstants.DEGREES_PER_SECOND);

        // Arm Povs
        if (speakerPositionButton.getState()) {
            arm.setArmToSpeakerPosition();
        }
        if (ampPositionButton.getState()) {
            arm.setArmToAmpPosition();
        }
        if (intakePositionButton.getState()) {
            arm.setArmToIntakePosition();
        }

	}

    @Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
	}
}
