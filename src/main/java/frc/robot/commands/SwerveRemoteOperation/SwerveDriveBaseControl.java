package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This can be the default command for the drivetrain, allowing for remote operation with a controller
 */
public abstract class SwerveDriveBaseControl extends Command {
   protected final SwerveDrivetrain drivetrain;
   protected final CommandGenericHID controller;

   /**
    * Creates a new SwerveDriveBaseControl Command.
    *
    * @param drivetrain       The drivetrain of the robot
    * @param driverController The device used to control drivetrain
    */
   public SwerveDriveBaseControl(SwerveDrivetrain drivetrain, CommandGenericHID driverController) {

      // save parameters
      this.controller = driverController;
      this.drivetrain = drivetrain;

      // Tell the command schedular we are using the drivetrain
      addRequirements(drivetrain);
   }

   /**
    * The initial subroutine of a command. Called once when the command is initially scheduled.
    * Puts all swerve modules to the default state, staying still and facing forwards.
    */
   @Override
   public void initialize() {
      drivetrain.enablePowerDriveMode();
      drivetrain.toDefaultStates();

      SmartDashboard.putBoolean("ControlActive", true);
   }

   /**
    * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
    */
   @Override
   public void execute() {
      final Pose2d robotPosition = drivetrain.getPosition();

      SmartDashboard.putNumber("PoseX", robotPosition.getX());
      SmartDashboard.putNumber("PoseY", robotPosition.getX());
      SmartDashboard.putNumber("PoseDegrees", robotPosition.getRotation().getDegrees());
   }

   /**
    * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and un-schedule it.
    * Always return false since we never want to end in this case.
    */
   @Override
   public boolean isFinished() {
      return false;
   }

   /**
    * The action to take when the command ends. Called when either the command finishes normally, or when it interrupted/canceled.
    * Should only happen in this case if we get interrupted.
    */
   @Override
   public void end(boolean interrupted) {
      drivetrain.disablePowerDriveMode();
      drivetrain.stop();

      SmartDashboard.putBoolean("ControlActive", false);
   }

   // --- Util ---

   /**
    * Utility method. Apply a deadzone to the joystick output to account for stick drift and small bumps.
    * 
    * @param joystickValue Value in [-1, 1] from joystick axis
    * @return {@code 0} if {@code |joystickValue| <= deadzone}, else the
    *         {@code joystickValue} scaled to the new control area
    */
   public static double applyJoystickDeadzone(double joystickValue, double deadzone) {
      if (Math.abs(joystickValue) <= deadzone) {
         // If the joystick |value| is in the deadzone than zero it out
         return 0;
      }

      // scale value from the range [0, 1] to (deadzone, 1]
      return joystickValue * (1 + deadzone) - Math.signum(joystickValue) * deadzone;
   }
}