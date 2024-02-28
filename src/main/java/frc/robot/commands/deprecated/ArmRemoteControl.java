package frc.robot.commands.deprecated;
// package frc.robot.commands;

// import frc.robot.Constants.ArmConstants;
// import frc.robot.inputs.OptionButtonInput;
// import frc.robot.subsystems.arm.ArmInterface;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj2.command.Command;

// public class ArmRemoteControl extends Command {
// 	private final ArmInterface arm;

// 	private final OptionButtonInput raiseArmButton;
// 	private final OptionButtonInput lowerArmButton;

// 	private final OptionButtonInput speakerPositionButton;
// 	private final OptionButtonInput ampPositionButton;
// 	private final OptionButtonInput intakePositionButton;

// 	public ArmRemoteControl(ArmInterface arm, OptionButtonInput raiseArmButton, OptionButtonInput lowerArmButton,
// 			OptionButtonInput speakerPositionButton, OptionButtonInput ampPositionButton,
// 			OptionButtonInput intakePositionButton) {
// 		this.arm = arm;

// 		this.raiseArmButton = raiseArmButton;
// 		this.lowerArmButton = lowerArmButton;

// 		this.speakerPositionButton = speakerPositionButton;
// 		this.ampPositionButton = ampPositionButton;
// 		this.intakePositionButton = intakePositionButton;

// 		addRequirements(arm);
// 	}

// 	@Override
// 	public void initialize() {
// 	}

// 	/**
// 	 * execute() is called repeatedly while a command is scheduled, about every
// 	 * 20ms.
// 	 */
// 	@Override
// 	public void execute() {

// 		// Arm Motor
// 		arm.changeArmAngleDegreesBy(Double.valueOf(raiseArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod
// 				* ArmConstants.DEGREES_PER_SECOND);
// 		arm.changeArmAngleDegreesBy(Double.valueOf(-lowerArmButton.getStateAsInt()) * TimedRobot.kDefaultPeriod
// 				* ArmConstants.DEGREES_PER_SECOND);

// 		// Arm Povs
// 		if (speakerPositionButton.getState()) {
// 			arm.setArmToSpeakerPosition();
// 		}
// 		if (ampPositionButton.getState()) {
// 			arm.setArmToAmpPosition();
// 		}
// 		if (intakePositionButton.getState()) {
// 			arm.setArmToIntakePosition();
// 		}

// 	}

// 	@Override
// 	public boolean isFinished() {
// 		return false;
// 	}

// 	@Override
// 	public void end(boolean interrupted) {
// 	}
// }
