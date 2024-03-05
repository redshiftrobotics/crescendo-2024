package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeShooterConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The intake & shooter system (mounted to the end of the arm)
 *
 * Other proposed names for this class were:
 * The Relocator
 * Non-Linear Accelarator
 * I. C. E.
 * Throngler
 * 
 * @author Aceius E.
 */
public class RealShooter extends IntakeShooter {
	/*
	 * 2 motors for flywheel
	 * 2 for intake
	 * neo550s, 2 sim motors,
	 */

	private final WPI_VictorSPX flywheel1;
	private final WPI_VictorSPX flywheel2;
	private long flywheelSpinUpStartTimeMillis;

	private final CANSparkMax intake;

	public RealShooter(int flywheel1Id, int flywheel2Id, int intakeID) {
		this.flywheel1 = new WPI_VictorSPX(flywheel1Id);
		this.flywheel2 = new WPI_VictorSPX(flywheel2Id);

		flywheelSpinUpStartTimeMillis = System.currentTimeMillis();

		this.intake = new CANSparkMax(intakeID, CANSparkLowLevel.MotorType.kBrushless);
		intake.setInverted(IntakeShooterConstants.INTAKE_REVERSE);
	}

	@Override
	public void setFlyWheelShooterSpeed(double speed) {
		speed = -speed;

		if (speed != flywheel1.get() || speed != flywheel2.get()) {
			flywheelSpinUpStartTimeMillis = System.currentTimeMillis();
		}
		
		flywheel1.set(speed);
		flywheel2.set(speed);
		
	}

	@Override
	public double getFlyWheelShooterSpinUpTimeSeconds() {
		long currentTimeMillis = System.currentTimeMillis();
		return Units.millisecondsToSeconds(currentTimeMillis - flywheelSpinUpStartTimeMillis);
	}

	@Override
	public void eject() {
		setIntakeGrabberSpeed(-1);
	}

	@Override
	public void stop() {
		setFlyWheelShooterSpeed(0);
		setIntakeGrabberSpeed(0);
	}

	@Override
	public void setIntakeGrabberSpeed(double speed) {
		intake.set(speed);
	}
}
