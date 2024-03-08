package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RealHang extends Hang {

	private final CANSparkMax leftMotor;
	private final CANSparkMax rightMotor;

	private final DigitalInput leftLimitSwitch;
	private final DigitalInput rightLimitSwitch;

	private double leftSpeed;
	private double rightSpeed;

	public RealHang(int leftMotorID, int rightMotorID, boolean leftMotorIsInverted, boolean rightMotorIsInverted,
			int leftLimitSwitchId, int rightLimitSwitchId) {
		leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
		rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);

		leftLimitSwitch = new DigitalInput(leftLimitSwitchId);
		rightLimitSwitch = new DigitalInput(rightLimitSwitchId);

		leftMotor.setInverted(leftMotorIsInverted);
		rightMotor.setInverted(rightMotorIsInverted);
		leftMotor.setIdleMode(IdleMode.kBrake);
		rightMotor.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		leftMotor.set(leftSpeed);
		rightMotor.set(rightSpeed);
		SmartDashboard.putBoolean("Is Left Arm at Bottom?", isAtBottomLeft());
		SmartDashboard.putBoolean("Is Right Arm at Bottom?", isAtBottomRight());
	}

	@Override
	public void setLeftSpeed(double speed) {
		this.leftSpeed = speed;
	}

	@Override
	public void setRightSpeed(double speed) {
		this.rightSpeed = speed;
	}

	@Override
	public boolean isAtBottomLeft() {
		return leftLimitSwitch.get();
	}

	@Override
	public boolean isAtBottomRight() {
		return rightLimitSwitch.get();
	}
}
