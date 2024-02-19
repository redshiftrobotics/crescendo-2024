package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

public class Arm extends SubsystemBase {

    private final CANSparkMax leftArmMotor;
    //private final CANcoder leftArmEncoder;

    //leftArmEncoder may be useless, as the right arm equivelent is used for the position for both.</p>

    private final CANSparkMax rightArmMotor;
    private final CANcoder rightArmEncoder;

    private final StatusSignal<Double> armPosition;

    private final PIDController armRaisePIDController;

    private Rotation2d armSetpoint;

    /** Constructor. Creates a new Arm Subsystem. */
    public Arm(int leftMotorId, int rightMotorId, int rightEncoderId, boolean areMotorsReversed) {

        leftArmMotor = new CANSparkMax(leftMotorId,MotorType.kBrushless);
        //leftArmEncoder = new CANcoder(leftEncoderId);

        rightArmMotor = new CANSparkMax(rightMotorId,MotorType.kBrushless);
        rightArmEncoder = new CANcoder(rightEncoderId);

        armRaisePIDController = new PIDController(
            ArmConstants.ELEVATION_PID_P,
            ArmConstants.ELEVATION_PID_I,
            ArmConstants.ELEVATION_PID_D
        );

        armPosition = rightArmEncoder.getAbsolutePosition();
        armSetpoint = Rotation2d.fromDegrees(ArmConstants.ARM_INTAKE_DEGREES);

        leftArmMotor.setIdleMode(IdleMode.kCoast);
        rightArmMotor.setIdleMode(IdleMode.kCoast);

        leftArmMotor.setInverted(areMotorsReversed);
        rightArmMotor.setInverted(!areMotorsReversed);

    }

    // public void setArmAngleDegrees(double desiredDegree) {
    //     //maximum should ALWAYS be a greater value then minimum
    //     if (desiredDegree < ArmConstants.MAXIMUM_ARM_DEGREES || ArmConstants.MINIMUM_ARM_DEGREES > desiredDegree) {
    //         armRotation2d = Rotation2d.fromDegrees(desiredDegree);
    //     }
    // }

     public void changeArmAngleDegreesBy(double desiredDegrees) {
        if (armSetpoint.getDegrees() < ArmConstants.MAXIMUM_ARM_DEGREES || ArmConstants.MINIMUM_ARM_DEGREES > armSetpoint.getDegrees()) {
            armSetpoint = Rotation2d.fromDegrees(armSetpoint.getDegrees() + desiredDegrees);
        }
    }


    public void setArmToAmpPosition() {
        armSetpoint = Rotation2d.fromDegrees(ArmConstants.ARM_AMP_SHOOTING_DEGREES);
    }

    public void setArmToSpeakerPosition() {
        armSetpoint = Rotation2d.fromDegrees(ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES);
    }

    public void setArmToIntakePosition() {
        armSetpoint = Rotation2d.fromDegrees(ArmConstants.ARM_INTAKE_DEGREES);
    }

    /**
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run


        double armSpeed = armRaisePIDController.calculate(armPosition.refresh().getValueAsDouble(),armSetpoint.getRotations());


        leftArmMotor.set(armSpeed);
        rightArmMotor.set(armSpeed);

        SmartDashboard.putNumber("Arm Degrees", Rotation2d.fromRotations(rightArmEncoder.getAbsolutePosition().getValueAsDouble()).getDegrees());
        SmartDashboard.putNumber("Arm Setpoint Degrees", armSetpoint.getDegrees());
            
        
    }
}
