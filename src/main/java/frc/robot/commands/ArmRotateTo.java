package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmRotateTo extends Command{

    private final double setpoint;
    private final Arm arm;

    public ArmRotateTo(Arm arm, double degree){
        this.setpoint = degree;
        this.arm = arm;

        addRequirements(arm);

    }

    @Override
    public void initialize() {
        arm.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return arm.isAtDesiredPosition();
    }
}
