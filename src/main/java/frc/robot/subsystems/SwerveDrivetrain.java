package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.IntFunction;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for full drive train of robot. Contains 4 {@link SwerveModule} subsystems.
 * 
 * @see <a href="https://youtu.be/X2UjzPi35gU">Swerve Drive Demo</a>
 */
public class SwerveDrivetrain extends SubsystemBase {
    /**
     * The SwerveDriveKinematics class is a useful tool that converts between a ChassisSpeeds object
     * and several SwerveModuleState objects, which contains velocities and angles for each swerve module of a swerve drive robot.
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics kinematics;

    /**
     * The SwerveDriveOdometry class can be used to track the position of a swerve drive robot on the field *
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
     */
    private final SwerveDriveOdometry odometry;

    /** 
     * The ChassisSpeeds object represents the speeds of a robot chassis.
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html#the-chassis-speeds-class
     */
    private ChassisSpeeds desiredSpeeds;

    // Our 4 swerve Modules
    private final SwerveModule moduleFL, moduleFR, moduleBL, moduleBR;
    private final SwerveModule[] modules;

    /**
     * The Gyroscope on the robot. It gives data on Pitch, Yaw, and Roll of robot, as well as many other things
     * @see https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
     * @see https://ibb.co/dJrL259
     */
    private final AHRS gyro;

    /**
     * Pose of robot. The pose is the current the X, Y and Rotation position of the robot, relative to the last reset.
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html#pose
     */
    private Pose2d pose;

    /**
     * Constructor the drivetrain subsystem.
     * 
     * @param gyro Gyroscope on robot, should be physically located near center of robot
     * @param swerveModuleFL Front left swerve module
     * @param swerveModuleFR Front right swerve module
     * @param swerveModuleBL Back left swerve module
     * @param swerveModuleBR Back right swerve module
     */
    public SwerveDrivetrain(AHRS gyro, SwerveModule swerveModuleFL, SwerveModule swerveModuleFR, SwerveModule swerveModuleBL, SwerveModule swerveModuleBR) {

        // save parameters
        this.gyro = gyro;

        moduleFL = swerveModuleFL;
        moduleFR = swerveModuleFR;
        moduleBL = swerveModuleBL;
        moduleBR = swerveModuleBR;

        modules = new SwerveModule[] {moduleFL, moduleFR, moduleBL, moduleBR};
        
        // create kinematics object using swerve module distance from center
        kinematics = new SwerveDriveKinematics(
            modulesMap(SwerveModule::getDistanceFromCenter, Translation2d[]::new)
        );

        // create starting state for odometry
        odometry = new SwerveDriveOdometry(
            kinematics,
            getHeading(),
            modulesMap(SwerveModule::getPosition, SwerveModulePosition[]::new)
        );

        for (SwerveModule module : modules) {
            addChild(module.getName(), module);
        }

        pose = new Pose2d();
    }

    // --- Pose Related Methods ---

    /**
     * This is the periodic function of the swerve drivetrain.
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        // use odometry to update the estimated pose
        pose = odometry.update(getHeading(), getWheelPositions());
    }

    /**
     * Get the pose of robot, as calculated by 
     * 
     * @return The current positions of the robot, contains translational and rotational elements.
     */
    public Pose2d getPosition() {
        return pose;
    }

    /**
     * Resets the robot's pose on the field in software.
     * Basically zeros out position.
     */
    public void resetPosition() {
        odometry.resetPosition(
            gyro.getRotation2d(),
            getWheelPositions(),
            pose
        );
    }


    // --- Action Methods ---


    /** Stop all swerve modules */
    public void stop() {
        modulesMap(SwerveModule::stop);
    }

    /** Put all swerve modules to default state, facing forward and staying still */
    public void toDefaultStates() {
        modulesMap(SwerveModule::toDefaultState);
    }


    // --- Chassis Speeds to Swerve Module State Methods ---


    /**
     * Get speeds of robot.
     * <p>vx: The velocity of the robot in the x (forward) direction in meter per second.</p>
     * <p>vy: The velocity of the robot in the y (sideways) direction in meter per second. (Positive values mean the robot is moving to the left).</p>
     * <p>omega: The angular velocity of the robot in radians per second.</p>
     * 
     * @return Speeds of drivetrain (from swerve modules)
     */
    public ChassisSpeeds getState() {
        // get all module states and convert them into chassis speeds
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            modulesMap(SwerveModule::getState, SwerveModuleState[]::new)
        );

        return speeds;
    }

    public void enablePowerDriveMode() {
        modulesMap(SwerveModule::enablePowerDriveMode);
    }
    
    public void disablePowerDriveMode() {
        modulesMap(SwerveModule::disablePowerDriveMode);
    }

    /**
     * Set robot relative speeds of robot.
     * <p>vx: The velocity of the robot in the x (forward) direction in meter per second.</p>
     * <p>vy: The velocity of the robot in the y (sideways) direction in meter per second. (Positive values mean the robot is moving to the left).</p>
     * <p>omega: The angular velocity of the robot in radians per second.</p>
     * 
     * @param speeds Desired speeds of drivetrain (using swerve modules)
     */
    public void setDesiredState(ChassisSpeeds speeds) {

        this.desiredSpeeds = speeds;

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(desiredSpeeds);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    /**
     * Set speeds of robot.
     * <li>vx: The velocity of the robot in the x (forward) direction in meter per second.</li>
     * <li>vy: The velocity of the robot in the y (sideways) direction in meter per second. (Positive values mean the robot is moving to the left).</li>
     * <li>omega: The angular velocity of the robot in radians per second.</li>
     * 
     * @see https://ibb.co/dJrL259
     * 
     * @param speeds Desired speeds of drivetrain (using swerve modules)
     * @param fieldRelative True if the robot is using a field relative coordinate system, false if using a robot relive coordinate system. If field relative, forward will be directly away from driver, no matter the rotation of the robot.
     * If robot relative, forward will be whatever direction the robot is facing in.
     */
    public void setDesiredState(ChassisSpeeds speeds, boolean fieldRelative) {

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
        }

        setDesiredState(speeds);
    }

    /**
     * Get the desired speeds of robot
     *
     * @return the value
     */
    public ChassisSpeeds getDesiredState() {
        return desiredSpeeds;
    }

    /**
     * Get all the swerve module positions
     * 
     * @return a {@link SwerveDriveWheelPositions} object which contains an array of all swerve module positions
     */
    public SwerveDriveWheelPositions getWheelPositions() {
        return new SwerveDriveWheelPositions(
            modulesMap(SwerveModule::getPosition, SwerveModulePosition[]::new)
        );
    }


    // --- Gyro methods ---


    /**
     * <p>Return the heading of the robot as a rotation in a 2D coordinate frame represented by a point on the unit circle (cosine and sine).</p>
     * 
     * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from 360 to 0 on the second time around.</p>
     * 
     * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the top.</p>
     * It needs to follow the NWU axis convention.
     * 
     * @return the current heading of the robot as a {@link Rotation2d}.
     * 
     * @see https://ibb.co/dJrL259 NWU Axis Convention 
     */
    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    /**
     * <p>Return the heading of the robot in as a rotation in a 3D coordinate frame represented by a quaternion.</p>
     * 
     * @return the current heading of the robot as a {@link Rotation3d}.
     */
    public Rotation3d getHeading3d() {
        return gyro.getRotation3d();
    }

    /** Reset the gyro. */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Zero the yaw of the gyro. Can only change zero yaw when sensor is done calibrating
     * 
     * @return successful?
     */
    public boolean zeroGyroYaw() {
        if (gyro.isCalibrating()) return false;

        gyro.zeroYaw();

        return true;
    }

    // --- Util ---

    /**
     * Utility method. Function to easily run a function on each swerve module
     * 
     * @param func function to run on each swerve module, takes one argument and returns nothing, operates via side effects.
     */
    private void modulesMap(Consumer<SwerveModule> func) {
        Arrays.stream(modules).forEach(func);
    }

    /**
     * Utility method. Function to easily run a function on each swerve module and collect results to array.
     * Insures that we don't mix up order of swerve modules, as this could lead to hard to spot bugs.
     * 
     * @param <T> type that is returned by function and should be collected
     * @param func function that gets some data off each swerve module
     * @param arrayInitializer constructor function for array to collect results in, use T[]::new
     * @return array of results from func.
     */
    private <T> T[] modulesMap(Function<? super SwerveModule, ? extends T> func, IntFunction<T[]> arrayInitializer) {
        // Private method with template argument T. Returns array of type T.
        // Takes in function that accepts a SwerveModule or something higher on the inheritance chain (For example: Object, SubsystemBase)
        // and returns something of type T or something lower on the inheritance chain. (For example if T is Object: Integer)
        // Also takes a T array initializer
        return Arrays.stream(modules).map(func).toArray(arrayInitializer);
    }   
}
