package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

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

	private final CANSparkMax intake;
	// private final CANSparkMax intake2; // might not exist

	public RealShooter(int flywheel1id, int flywheel2id, int intake1id, int intake2id) {
		this.flywheel1 = new WPI_VictorSPX(flywheel1id);
		this.flywheel2 = new WPI_VictorSPX(flywheel2id);

		this.intake = new CANSparkMax(intake1id, CANSparkLowLevel.MotorType.kBrushless);
		intake.setInverted(IntakeShooterConstants.INTAKE_REVERSE);
	}

	public void setFlyWheelSpeed(double speed) {
		flywheel1.set(-speed);
		flywheel2.set(-speed);
	}

	public void startFlyWheels() {
		setFlyWheelSpeed(1);
	}

	public void stopFlywheels() {
		setFlyWheelSpeed(0);
	}

	public void reverseFlywheel() {
		setFlyWheelSpeed(-0.075);
	}

	public void setIntakeSpeed(double speed) {
		intake.set(speed);
		// intake2.set(speed);
	}

	public void intake() {
		setIntakeSpeed(1);
	}

	public void intakeReverse() {
		setIntakeSpeed(-1);
	}

	public void stopIntake() {
		setIntakeSpeed(0);
	}

}
