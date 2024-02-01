package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

// How to make Subsystem (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html



public class Arm extends SubsystemBase {

    private final CANSparkMax leftArmMotor;
    private final CANcoder leftArmEncoder;
    //<p>leftArmEncoder may be useless, as the right arm equivelent is the position for both.</p>
    private final CANSparkMax rightArmMotor;
    private final CANcoder rightArmEncoder;

    private final StatusSignal<Double> armPosition;

    private final PIDController armRaisePIDController;

    private Rotation2d armRotation2d;

    

    /** Constructor. Creates a new ExampleSubsystem. */
    public Arm(int leftMotorId, int leftEncoderId, int rightMotorId, int rightEncoderId) {

        leftArmMotor = new CANSparkMax(leftMotorId,MotorType.kBrushless);
        leftArmEncoder = new CANcoder(leftEncoderId);

        rightArmMotor = new CANSparkMax(rightMotorId,MotorType.kBrushless);
        rightArmEncoder = new CANcoder(rightEncoderId);

        armRaisePIDController = new PIDController(
            0,
            0,
            0
            );

        armPosition = rightArmEncoder.getAbsolutePosition();

    }

    public void setArmAngleRadians(double desiredRadian) {
        //maximum should ALWAYS be a greater value then minimum
        if (desiredRadian < ArmConstants.MAXIMUM_ARM_RADIANS || ArmConstants.MINIMUM_ARM_RADIANS > desiredRadian) {
            armRotation2d = Rotation2d.fromRadians(desiredRadian);
        }
    }

    /**
     * This method is called periodically by the CommandScheduler, about every 20ms.
     * It should be used for updating subsystem-specific state that you don't want to offload to a Command.
     * Try to avoid "doing to much" in this method (for example no driver control here).
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        final double armSpeed = armRaisePIDController.calculate(armPosition.refresh().getValueAsDouble(),armRotation2d.getRotations());
        if(!(armSpeed == 0)){
            leftArmMotor.set(armSpeed);
            rightArmMotor.set(armSpeed);
        }
        
    }
}
