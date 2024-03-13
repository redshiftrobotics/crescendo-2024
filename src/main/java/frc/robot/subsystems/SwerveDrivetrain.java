package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.IntFunction;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;

/**
 * Subsystem for full drive train of robot. Contains 4 {@link SwerveModule}
 * subsystems.
 * 
 * @see <a href="https://youtu.be/X2UjzPi35gU">Swerve Drive Demo</a>
 */
public class SwerveDrivetrain extends SubsystemBase {
	/**
	 * The SwerveDriveKinematics class is a useful tool that converts between a
	 * ChassisSpeeds object
	 * and several SwerveModuleState objects, which contains velocities and angles
	 * for each swerve module of a swerve drive robot.
	 * 
	 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
	 */
	private final SwerveDriveKinematics kinematics;

	/**
	 * The SwerveDriveOdometry class can be used to track the position of a swerve drive robot on the field
	 *
	 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
	 */
	private final SwerveDriveOdometry poseOdometry;

	/** Swerve module */
	private final SwerveModule moduleFL, moduleFR, moduleBL, moduleBR;
	private final SwerveModule[] modules;

	/**
	 * The Gyroscope on the robot. It gives data on Pitch, Yaw, and Roll of robot,
	 * as well as many other things
	 * 
	 * @see https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
	 * @see https://ibb.co/dJrL259
	 */
	private final AHRS gyro;

	/**
	 * Amount to add to gyro position for field relative drive and SmartDashboard display
	 */
	private Rotation2d frontOffset = new Rotation2d();

	/**
	 * Pose of robot. The pose is the current the X, Y and Rotation position of the
	 * robot, relative to the last reset.
	 * It is updated every 20ms in periodic.
	 * 
	 * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html#pose
	 */
	private Pose2d pose = new Pose2d();

	/**
	 * Desired pose of robot. The desired pose is the X, Y and Rotation the robot
	 * wants to be in, relative to the last reset.
	 * It can be set to null to not have any desired pose.
	 * 
	 * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html#pose
	 */
	private Pose2d desiredPose;

	/** The PID controller to get robot to desired pose */
	private final PIDController xController, yController, rotationController;

	/**
	 * Constructor the drivetrain subsystem.
	 * 
	 * @param gyro           Gyroscope on robot, should be physically located near
	 *                       center of robot
	 * @param swerveModuleFL Front left swerve module
	 * @param swerveModuleFR Front right swerve module
	 * @param swerveModuleBL Back left swerve module
	 * @param swerveModuleBR Back right swerve module
	 */
	public SwerveDrivetrain(
			AHRS gyro,
			SwerveModule swerveModuleFL, SwerveModule swerveModuleFR,
			SwerveModule swerveModuleBL, SwerveModule swerveModuleBR) {

		// save parameters
		this.gyro = gyro;

		moduleFL = swerveModuleFL;
		moduleFR = swerveModuleFR;
		moduleBL = swerveModuleBL;
		moduleBR = swerveModuleBR;

		modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };

		// create kinematics object using swerve module distance from center
		kinematics = new SwerveDriveKinematics(
				modulesMap(SwerveModule::getDistanceFromCenter, Translation2d[]::new));

		// Create pose estimator, like odometry but cooler
		poseOdometry = new SwerveDriveOdometry(
				kinematics,
				getHeading(),
				getWheelPositions().positions,
				pose);

		// set up name and children for sendable registry
		setName(toString());
		for (SwerveModule module : modules) {
			addChild(module.getName(), module);
		}

		// set up PID controllers for desired pose
		xController = new PIDController(RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I, RobotMovementConstants.TRANSLATION_PID_D);
		xController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);

		yController = new PIDController(RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I, RobotMovementConstants.TRANSLATION_PID_D);
		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);

		rotationController = new PIDController(RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I, RobotMovementConstants.ROTATION_PID_D);
		rotationController.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
	}

	// --- Pose Related Methods ---

	/**
	 * This is the periodic function of the swerve drivetrain, called periodically
	 * by the CommandScheduler, about every 20ms.
	 */
	@Override
	public void periodic() {
		pose = poseOdometry.update(
				getHeading(),
				getWheelPositions());

		// if (visionSystem != null) {
		// 	Optional<EstimatedRobotPose> estimatedPose =visionSystem.getEstimatedGlobalPose();
		// 	if (estimatedPose.isPresent()) {
		// 		poseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(),
		// 		estimatedPose.get().timestampSeconds);
		// 	}
		// }

		if (desiredPose != null) {
			// Calculate our robot speeds with the PID controllers
			final ChassisSpeeds speeds = new ChassisSpeeds(
					xController.calculate(pose.getX(), desiredPose.getX()),
					yController.calculate(pose.getY(), desiredPose.getY()),
					rotationController.calculate(pose.getRotation().getRadians(),
							desiredPose.getRotation().getRadians()));

			// Set those speeds
			setDesiredState(speeds);
		}
	}

	/**
	 * Get the pose of robot, as calculated by odometry from gyro and distances
	 * swerve modules have traveled
	 * 
	 * @return The current positions of the robot, contains translational and
	 *         rotational elements.
	 */
	public Pose2d getPosition() {
		return pose;
	}

	/**
	 * Resets the robot's pose on the field in software.
	 * Basically zeros out position.
	 */
	public void resetPosition() {
		poseOdometry.resetPosition(
				getHeading(),
				getWheelPositions(),
				pose);
	}

	/**
	 * <p>
	 * This drives relative to the robot starting position,
	 * so a pose of +2x and +1y will drive to the position 2 meters forward and 1
	 * meter left of whether the robot started,
	 * where forward is whatever direction the robot started in
	 * </p>
	 * 
	 * <p>
	 * The last place the drivetrain position was reset counts as the starting
	 * position
	 * </p>
	 *
	 * @param desiredPose the pose the robot will try to drive to
	 */
	public void setDesiredPosition(Pose2d desiredPose) {
		this.desiredPose = desiredPose;
	}

	/** Gets the desired position, returns null if the pose has been cleared */
	public Pose2d getDesiredPose() {
		return desiredPose;
	}

	/**
	 * Sets desired position to null, stops robot from continue to try and get to
	 * the last set pose
	 */
	public void clearDesiredPosition() {
		setDesiredPosition(null);
	}

	/**
	 * Checks whether drivetrain is at the desired pose
	 * 
	 * @return are all drive PID controllers within tolerance of their setpoints
	 */
	public boolean isAtDesiredPosition() {
		return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
	}

	// --- Action Methods ---

	/** Stop all swerve modules, clears desired position */
	public void stop() {
		clearDesiredPosition();
		modulesMap(SwerveModule::stop);
	}

	/**
	 * Put all swerve modules to default state, facing forward and staying still.
	 * Also clears desired position.
	 */
	public void toDefaultStates() {
		clearDesiredPosition();
		modulesMap(SwerveModule::toDefaultState);
	}

	/**
	 * Put all swerve modules to inward state, making the swerve modules face in a X
	 * pattern. This prevents robot from slipping around. Also clears desired
	 * position
	 */
	public void brakeMode() {
		clearDesiredPosition();
		modulesMap(SwerveModule::toInwardPosition);
	}

	// --- Chassis Speeds to Swerve Module States methods ---

	/**
	 * Get field relative speeds of robot.
	 * 
	 * @return Speeds of drivetrain (from swerve modules)
	 */
	public ChassisSpeeds getState() {
		// get all module states and convert them into chassis speeds
		return kinematics.toChassisSpeeds(getWheelStates());
	}

	/**
	 * Get speeds of robot.
	 * 
	 * @param fieldRelative True if the robot is using a field relative coordinate
	 *                      system,
	 *                      false if using a robot relive coordinate system.
	 * @return Speeds of drivetrain (from swerve modules)
	 */
	public ChassisSpeeds getState(boolean fieldRelative) {
		if (fieldRelative)
			return ChassisSpeeds.fromRobotRelativeSpeeds(getState(), getHeading());
		return getState();
	}

	/**
	 * Set robot relative speeds of robot using default speeds units.
	 * 
	 * @param speeds Desired speeds of drivetrain (using swerve modules)
	 */
	public void setDesiredState(ChassisSpeeds speeds) {
		setDesiredState(speeds, false);
	}

	public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
		setDesiredState(speeds, fieldRelative);
	}

	/**
	 * Set speeds of robot.
	 * 
	 * <p>
	 * Vx: the velocity of the robot in the x (forward) direction in meter per
	 * second. (Positive is forward)
	 * Vy: the velocity of the robot in the y (sideways) direction in meter per
	 * second. (Positive is left).
	 * Omega: the angular velocity of the robot in radians per second. (Positive is
	 * counterclockwise)
	 * 
	 * <p>
	 * If field relative, forward will be directly away from driver, no matter the
	 * rotation of the robot.
	 * If robot relative, forward will be whatever direction the robot is facing in.
	 * 
	 * @param speeds         Desired speeds of drivetrain (using swerve modules)
	 * @param fieldRelative  True if the robot is using a field relative coordinate
	 *                       system,
	 *                       false if using a robot relive coordinate system
	 */
	public void setDesiredState(ChassisSpeeds speeds, boolean fieldRelative) {

		SmartDashboard.putNumber("SpeedX", speeds.vxMetersPerSecond);
		SmartDashboard.putNumber("SpeedY", speeds.vyMetersPerSecond);
		SmartDashboard.putNumber("Spin", speeds.omegaRadiansPerSecond);

		if (fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		speeds = ChassisSpeeds.discretize(speeds, TimedRobot.kDefaultPeriod);

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDrivetrainConstants.MAX_OBTAINABLE_SPEED);

		for (int i = 0; i < modules.length; i++) {
			modules[i].setDesiredState(states[i]);
		}
	}

	/**
	 * Get all the swerve module positions
	 * 
	 * @return a {@link SwerveDriveWheelPositions} object which contains an array of
	 *         all swerve module positions
	 */
	public SwerveDriveWheelPositions getWheelPositions() {
		return new SwerveDriveWheelPositions(
				modulesMap(SwerveModule::getPosition, SwerveModulePosition[]::new));
	}

	/**
	 * Get all the swerve module states
	 * 
	 * @return a {@link SwerveDriveWheelPositions} object which contains an array of
	 *         all swerve module positions
	 */
	public SwerveDriveWheelStates getWheelStates() {
		return new SwerveDriveWheelStates(
				modulesMap(SwerveModule::getState, SwerveModuleState[]::new));
	}

	// --- Gyro methods ---

	/**
	 * Gyro Method
	 * 
	 * <p>
	 * Return the heading of the robot as a rotation in a 2D coordinate frame
	 * represented by a point on the unit circle (cosine and sine).
	 * 
	 * <p>
	 * The angle is continuous, that is it will continue from 360 to 361 degrees.
	 * This allows algorithms that wouldn't want to see a discontinuity in the gyro
	 * output as it sweeps past from 360 to 0 on the second time around.
	 * 
	 * <p>
	 * The angle is expected to increase as the gyro turns counterclockwise when
	 * looked at from the top.
	 * 
	 * It follows the NWU axis convention.
	 * 
	 * @return the current heading of the robot as a {@link Rotation2d}.
	 * 
	 * @see https://ibb.co/dJrL259 NWU Axis Convention
	 */
	public Rotation2d getHeading() {
		return gyro.getRotation2d().plus(frontOffset);
	}

	/**
	 * Gyro Method
	 * 
	 * <p>
	 * Return the heading of the robot in as a rotation in a 3D coordinate frame
	 * represented by a quaternion.
	 * </p>
	 * 
	 * @return the current heading of the robot as a {@link Rotation3d}.
	 */
	public Rotation3d getHeading3d() {
		return gyro.getRotation3d();
	}

	
	/**
	 * Set amount to add to gyro position for field relative drive and SmartDashboard display
	 * 
	 * @param frontOffset rotation2d to add
	 */
	public void setFrontOffset(Rotation2d frontOffset) {
		this.frontOffset = frontOffset;
	}

	// --- Util ---

	/** Update SmartDashboard with robot values */
	public void updateSmartDashboard() {
		// Position display
		final Pose2d robotPosition = getPosition();

		SmartDashboard.putNumber("PoseX", robotPosition.getX());
		SmartDashboard.putNumber("PoseY", robotPosition.getY());
		SmartDashboard.putNumber("PoseDegrees", robotPosition.getRotation().getDegrees());

		// Speed and Heading
		final ChassisSpeeds currentSpeeds = getState();
		final double speedMetersPerSecond = Math
				.sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

		final double metersPerSecondToMilesPerHourConversion = 2.237;
		SmartDashboard.putNumber("Robot MPH", speedMetersPerSecond * metersPerSecondToMilesPerHourConversion);
		SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());

		// final boolean hasTargetPose = desiredPose != null;
		// final Pose2d targetPose = hasTargetPose ? desiredPose : new Pose2d();
		// SmartDashboard.putBoolean("tPoseActive", hasTargetPose);
		// if (hasTargetPose) {
		// SmartDashboard.putNumber("tPoseX", targetPose.getX());
		// SmartDashboard.putNumber("tPoseY", targetPose.getY());
		// SmartDashboard.putNumber("tPoseDegrees",
		// targetPose.getRotation().getDegrees());
		// }
	}

	/**
	 * Utility method. Function to easily run a function on each swerve module
	 * 
	 * @param func function to run on each swerve module, takes one argument and
	 *             returns nothing, operates via side effects.
	 */
	private void modulesMap(Consumer<SwerveModule> func) {
		Arrays.stream(modules).forEach(func);
	}

	/**
	 * Utility method. Function to easily run a function on each swerve module and collect results to array.
	 * Insures that we don't mix up order of swerve modules, as this could lead to hard-to-spot bugs.
	 * 
	 * @param <T>              type that is returned by function and should be
	 *                         collected
	 * @param func             function that gets some data off each swerve module
	 * @param arrayInitializer constructor function for array to collect results in,
	 *                         use T[]::new
	 * @return array of results from func.
	 */
	private <T> T[] modulesMap(Function<? super SwerveModule, ? extends T> func, IntFunction<T[]> arrayInitializer) {
		// Private method with template argument T. Returns array of type T.
		// Takes in function that accepts a SwerveModule or something higher on the
		// inheritance chain (For example: Object, SubsystemBase)
		// and returns something of type T or something lower on the inheritance chain.
		// (For example if T is Object: Integer)
		// Also takes a T array initializer (T[]::new)
		return Arrays.stream(modules).map(func).toArray(arrayInitializer);
	}

	@Override
	public String toString() {
		return "SwerveDrivetrain";
	}
}
