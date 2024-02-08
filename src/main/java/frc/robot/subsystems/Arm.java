package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;

// How to make Subsystem (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html



public class Arm extends SubsystemBase {

    private final CANSparkMax leftArmMotor;
    //private final CANcoder leftArmEncoder;

    //leftArmEncoder may be useless, as the right arm equivelent is used for the position for both.</p>

    private final CANSparkMax rightArmMotor;
    private final CANcoder rightArmEncoder;

    private final StatusSignal<Double> armPosition;

    private final PIDController armRaisePIDController;

    private Rotation2d armRotation2d;

    

    /** Constructor. Creates a new Arm Subsystem. */
    public Arm(int leftMotorId, int rightMotorId, int rightEncoderId) {

        leftArmMotor = new CANSparkMax(leftMotorId,MotorType.kBrushless);
        //leftArmEncoder = new CANcoder(leftEncoderId);

        rightArmMotor = new CANSparkMax(rightMotorId,MotorType.kBrushless);
        rightArmEncoder = new CANcoder(rightEncoderId);

        armRaisePIDController = new PIDController(
            0,
            0,
            0
            );

        armPosition = rightArmEncoder.getAbsolutePosition();

        leftArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setIdleMode(IdleMode.kBrake);

    }

    public void setArmAngleDegrees(double desiredDegree) {
        //maximum should ALWAYS be a greater value then minimum
        if (desiredDegree < ArmConstants.MAXIMUM_ARM_DEGREES || ArmConstants.MINIMUM_ARM_DEGREES > desiredDegree) {
            armRotation2d = Rotation2d.fromDegrees(desiredDegree);
        }
    }

    public void setArmToAmpPosition() {
        armRotation2d = Rotation2d.fromRadians(ArmConstants.ARM_AMP_SHOOTING_DEGREES);
    }

    public void setArmToSpeakerPosition() {
        armRotation2d = Rotation2d.fromRadians(ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES);
    }

    public void setArmToIntakePosition() {
        armRotation2d = Rotation2d.fromRadians(ArmConstants.ARM_INTAKE_DEGREES);
    }

    /**
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        final double armSpeed = armRaisePIDController.calculate(armPosition.refresh().getValueAsDouble(),armRotation2d.getRotations());
        leftArmMotor.set(armSpeed);
        rightArmMotor.set(armSpeed);
            
        
    }
}
