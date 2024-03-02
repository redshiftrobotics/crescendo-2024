package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;

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
public class IntakeShooter extends SubsystemBase {
	/*
	 * 2 motors for flywheel
	 * 2 for intake
	 * neo550s, 2 sim motors,
	 */

	private final Talon flywheel1; // AndyMark CIM, doublecheck this
	private final Talon flywheel2;
	private final CANSparkMax intake1;
	private final CANSparkMax intake2; // might not exist

	public IntakeShooter(int flywheel1id, int flywheel2id, int intake1id, int intake2id) {
		this.flywheel1 = new Talon(flywheel1id);
		this.flywheel2 = new Talon(flywheel2id);
		
		flywheel1.setInverted(IntakeShooterConstants.FLYWHEEL_REVERSE);
		flywheel1.setInverted(IntakeShooterConstants.FLYWHEEL_REVERSE);
		
		this.intake1 = new CANSparkMax(intake1id, CANSparkLowLevel.MotorType.kBrushless);
		this.intake2 = new CANSparkMax(intake2id, CANSparkLowLevel.MotorType.kBrushless);

		intake1.setInverted(IntakeShooterConstants.INTAKE_REVERSE);
		intake2.setInverted(IntakeShooterConstants.INTAKE_REVERSE);
	}

	public void setFlyWheelSpeed(double speed) {
		flywheel1.set(speed);
		flywheel2.set(speed);
	}

	public void startFlyWheels() {
		setFlyWheelSpeed(1);
	}
	
	public void stopFlywheels() {
		setFlyWheelSpeed(0);
	}

	public void setIntakeSpeed(double speed) {
		intake1.set(speed);
		intake2.set(speed);
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
