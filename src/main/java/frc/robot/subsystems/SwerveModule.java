package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;
    private final RelativeEncoder driveEncoder;
    private final CANcoder steeringEncoder;
    private final RelativeEncoder steeringRelativeEncoder;
    private final SparkPIDController drivePIDController;
    private final PIDController steeringPIDController;
    //private final double offset;
    private final StatusSignal<Double> steeringPosition;
    private SwerveModuleState desiredState;

    public SwerveModule(int driveMotorId, int steeringMotorId, int steeringAbsoluteEncoderId, double steeringEncoderZero) {
        driveMotor = new CANSparkMax(driveMotorId,MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorId,MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = new CANcoder(steeringAbsoluteEncoderId);
        steeringRelativeEncoder = steeringMotor.getEncoder();

        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(SwerveModuleConstants.DRIVE_PID_P);
        drivePIDController.setI(SwerveModuleConstants.DRIVE_PID_I);
        drivePIDController.setD(SwerveModuleConstants.DRIVE_PID_D);
        drivePIDController.setFF(SwerveModuleConstants.DRIVE_PID_FF);
        drivePIDController.setIZone(SwerveModuleConstants.DRIVE_PID_IZone);
        drivePIDController.setIAccum(SwerveModuleConstants.DRIVE_PID_IAccum);
        drivePIDController.setOutputRange(-SwerveModuleConstants.MAX_SPEED, SwerveModuleConstants.MAX_SPEED);
        
        CANcoderConfigurator configurator = steeringEncoder.getConfigurator();
        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.MagnetOffset = steeringEncoderZero;
        configurator.apply(configuration);

        steeringPIDController = new PIDController(
            SwerveModuleConstants.STEERING_PID_P,
            SwerveModuleConstants.STEERING_PID_I,
            SwerveModuleConstants.STEERING_PID_D
        );
        steeringPIDController.enableContinuousInput(0, 360);

        steeringPosition = steeringEncoder.getPosition();
    }

    public void stop() {
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveMetersPerSecond(),
            Rotation2d.fromRotations(getSteeringAngle())
        );
    }

    public void setState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    private double getDriveMetersPerSecond() {
        return SwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS * driveEncoder.getVelocity() / 60;
    }

    private double getSteeringAngle() {
        return steeringPosition.refresh().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (desiredState != null) {
            //currentAngle = getState();
        }
    }
}
