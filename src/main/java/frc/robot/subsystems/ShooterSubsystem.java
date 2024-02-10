package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

// How to make Subsystem: https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html (ignore image instructions, code is out of date, just look at written general instructions)
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

/**
 * How to make a Subsystem:
 * 1. Copy this file, remember that class name has to match
 */

public class ShooterSubsystem extends SubsystemBase {

	private final CANSparkMax greenMotor;
	private final CANSparkMax greyMotorOne;
	private final CANSparkMax greyMotorTwo;

	private final PIDController ShooterPIDController;

	/** Constructor. Creates a new ExampleSubsystem. */
	public ShooterSubsystem(int greyMotorOneId, int greyMotorTwoId, int greenMotorId, int SHOOTER_PID_P,
			int SHOOTER_PID_I, int SHOOTER_PID_D) {

		greyMotorOne = new CANSparkMax(greyMotorOneId, MotorType.kBrushless);
		// Grey motor one is the top of the pair
		greyMotorTwo = new CANSparkMax(greyMotorTwoId, MotorType.kBrushless);
		greenMotor = new CANSparkMax(greenMotorId, MotorType.kBrushless);

		ShooterPIDController = new PIDController(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D);
	}

	public void engageIntake() {
		greenMotor.set(ShooterConstants.intakeSpeed);
	}

	public void engageOutput() {
		greyMotorOne.set(ShooterConstants.outputSpeed);
		greyMotorTwo.set(ShooterConstants.outputSpeed);
	}

	/**
	 * This method is called periodically by the CommandScheduler, about every 20ms.
	 * It should be used for updating subsystem-specific state that you don't want
	 * to offload to a Command.
	 * 
	 * 
	 * here).
	 */
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
