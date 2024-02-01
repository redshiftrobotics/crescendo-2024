package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command testingAuto(SwerveDrivetrain drivetrain) {
        return Commands.sequence(
            new AutoDriveTo(drivetrain, new Translation2d(1, 0))
            // new WaitCommand(1),
            // new AutoDriveTo(drivetrain, new Translation2d(-1, 0)),
        );
    }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
