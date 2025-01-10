package frc.robot.subsystems.hang;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RealHang extends Hang {

	private final SparkMax motor;
	// private final DigitalInput leftLimitSwitch;
	private final String name;

	public RealHang(int motorID, boolean motorIsInverted, int rightLimitSwitchId, String name) {
		motor = new SparkMax(motorID, MotorType.kBrushless);

		SparkMaxConfig motorCOnfig = new SparkMaxConfig();
		motorCOnfig.idleMode(IdleMode.kBrake);
		motorCOnfig.inverted(motorIsInverted);

		motor.configure(motorCOnfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// leftLimitSwitch = new DigitalInput(rightLimitSwitchId);

		this.name = name;

		setName(name + "Hang");
	}

	public RealHang(int motorID, boolean motorIsInverted, int rightLimitSwitchId) {
		this(motorID, motorIsInverted, rightLimitSwitchId, String.valueOf(motorID));
	}

	@Override
	public void periodic() {
		SmartDashboard.putString(name + "Hang", motor.get() + "p" + (isAtBottom() ? "Down" : "Up"));
		SmartDashboard.putBoolean(name + "Hang Is Down", isAtBottom());
	}

	@Override
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	@Override
	public boolean isAtBottom() {
		return true;
		// return leftLimitSwitch.get();
	}
}
