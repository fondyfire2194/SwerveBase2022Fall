// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

  public final HashMap<ModulePosition, SwerveModuleSparkMax> m_swerveModules = new HashMap<>(

      Map.of(

          ModulePosition.FRONT_LEFT,
          new SwerveModuleSparkMax(ModulePosition.FRONT_LEFT,
              CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
              DriveConstants.kBackLeftDriveMotorReversed,
              DriveConstants.kBackLeftTurningMotorReversed,
              CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET),

          ModulePosition.FRONT_RIGHT,
          new SwerveModuleSparkMax(
              ModulePosition.FRONT_RIGHT,
              CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
              DriveConstants.kFrontRightDriveMotorReversed,
              DriveConstants.kFrontRightTurningMotorReversed,
              CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET),

          ModulePosition.BACK_LEFT,
          new SwerveModuleSparkMax(ModulePosition.BACK_LEFT,
              CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
              DriveConstants.kBackLeftDriveMotorReversed,
              DriveConstants.kBackLeftTurningMotorReversed,
              CanConstants.BACK_LEFT_MODULE_STEER_OFFSET),

          ModulePosition.BACK_RIGHT,
          new SwerveModuleSparkMax(
              ModulePosition.BACK_RIGHT,
              CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
              DriveConstants.kBackRightDriveMotorReversed,
              DriveConstants.kBackRightTurningMotorReversed,
              CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET)));
  // The gyro sensor

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  private PIDController m_xController = new PIDController(DriveConstants.kP_X, 0, DriveConstants.kD_X);
  private PIDController m_yController = new PIDController(DriveConstants.kP_Y, 0, DriveConstants.kD_Y);
  private ProfiledPIDController m_turnController = new ProfiledPIDController(
      DriveConstants.kP_Theta, 0,
      DriveConstants.kD_Theta,
      Constants.TrapezoidConstants.kThetaControllerConstraints);

  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      getHeadingRotation2d(),
      new Pose2d(),
      kSwerveKinematics,
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.05),
      VecBuilder.fill(0.1, 0.1, 0.1));

  private boolean showOnShuffleboard = true;

  private SimDouble m_simAngle;// navx sim

  private double m_simYaw;

  public double throttleValue;

  public double targetAngle;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_gyro.reset();

    resetModuleEncoders();

    setIdleMode(true);

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")

  // move the robot from gamepad
  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
    throttle *= DriveConstants.kMaxSpeedMetersPerSecond;
    strafe *= DriveConstants.kMaxSpeedMetersPerSecond;
    rotation *= DriveConstants.kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            throttle, strafe, rotation, getHeadingRotation2d())
        : new ChassisSpeeds(throttle, strafe, rotation);

    Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
        .of(kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

  }

  public void updateOdometry() {
    m_odometry.update(
        getHeadingRotation2d(),
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Translation2d modulePositionFromChassis = DriveConstants.kModuleTranslations
          .get(module.getModulePosition())
          .rotateBy(getHeadingRotation2d())
          .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getEstimatedPosition();
  }

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public void setOdometry(Pose2d pose) {

    m_odometry.resetPosition(pose, pose.getRotation());
    m_gyro.reset();

  }

  public SwerveModuleSparkMax getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((m_gyro.getAngle()), 360);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : m_swerveModules.keySet()) {
      map.put(i, m_swerveModules.get(i).getState());
    }
    return map;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void resetModuleEncoders() {
    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules))
      module.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_gyro.reset();
    // m_gyro.setAngleAdjustment(0);

  }

  public Translation2d getTranslation() {
    return getPoseMeters().getTranslation();
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public ProfiledPIDController getThetaPidController() {
    return m_turnController;
  }

  public double getX() {
    return getTranslation().getX();
  }

  public double getY() {
    return getTranslation().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public void setIdleMode(boolean brake) {
    for (SwerveModuleSparkMax module : ModuleMap.orderedValuesList(m_swerveModules)) {
      module.setDriveBrakeMode(brake);
      module.setTurnBrakeMode(brake);
    }

  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeedSim = kSwerveKinematics.toChassisSpeeds(
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;

    temp += m_simAngle.get();

    m_simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  public void turnModule(ModulePosition mp, double speed) {
    getSwerveModule(mp).turnMotorMove(speed);
  }

  public void positionTurnModule(ModulePosition mp, double angle) {
    getSwerveModule(mp).positionTurn(angle);
  }

  public void driveModule(ModulePosition mp, double speed) {
    getSwerveModule(mp).driveMotorMove(speed);
  }

  public boolean getTurnInPosition(ModulePosition mp, double targetAngle) {
    return getSwerveModule(mp).turnInPosition(targetAngle);
  }

  public double getAnglefromThrottle() {

    return 180 * throttleValue;
  }

}
