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

	public void startFlyWheels() {
		flywheel1.set(1);
		flywheel2.set(1);
	}
	
	public void stopFlywheels() {
		flywheel1.stopMotor();
		flywheel2.stopMotor();
	}
	
	public void intake() {
		intake1.set(1);
		intake1.set(1);
	}

	public void intakeReverse() {
		intake1.set(-1);
		intake1.set(-1);
	}

	public void stopIntake() {
		intake1.stopMotor();
		intake1.stopMotor();
	}

}
