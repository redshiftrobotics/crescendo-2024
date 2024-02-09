package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitchArmSubsystem extends SubsystemBase {
	private final CANSparkMax motor1;
	private final CANSparkMax motor2;
	private final DigitalInput upperLimitSwitch;
	private final DigitalInput lowerLimitSwitch;
	// false = lower, true = upper
	private boolean desiredState = true;

	public LimitSwitchArmSubsystem(int motor1ID, int motor2ID, int upperLimitSwitchID, int lowerLimitSwitchID,
			boolean isMotor1Inverted, boolean isMotor2Inverted) {
		motor1 = new CANSparkMax(motor1ID, MotorType.kBrushless);
		motor2 = new CANSparkMax(motor2ID, MotorType.kBrushless);
		upperLimitSwitch = new DigitalInput(upperLimitSwitchID);
		lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);
		motor1.setInverted(isMotor1Inverted);
		motor2.setInverted(isMotor2Inverted);
	}

	public void setState(boolean newDesiredState) {
		desiredState = newDesiredState;
	}

	@Override
	public void periodic() {
		if (desiredState) {
			if (upperLimitSwitch.get()) {
				motor1.stopMotor();
				motor2.stopMotor();
			} else {
				motor1.set(0.05);
				motor2.set(0.05);
			}
		} else {
			if (lowerLimitSwitch.get()) {
				motor1.stopMotor();
				motor2.stopMotor();
			} else {
				motor1.set(-0.05);
				motor2.set(-0.05);
			}

		}

	}

}
