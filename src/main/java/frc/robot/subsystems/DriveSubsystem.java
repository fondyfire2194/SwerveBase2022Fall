// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.ModuleMap;
import frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

  final HashMap<ModulePosition, SwerveModuleSparkMax4201> m_swerveModules = new HashMap<>(

      Map.of(

          ModulePosition.FRONT_LEFT,
          new SwerveModuleSparkMax4201(ModulePosition.FRONT_LEFT,
              CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
              DriveConstants.kBackLeftDriveMotorReversed,
              DriveConstants.kBackLeftTurningMotorReversed,
              CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET),

          ModulePosition.FRONT_RIGHT,
          new SwerveModuleSparkMax4201(
              ModulePosition.FRONT_RIGHT,
              CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
              DriveConstants.kFrontRightDriveMotorReversed,
              DriveConstants.kFrontRightTurningMotorReversed,
              CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET),

          ModulePosition.BACK_LEFT,
          new SwerveModuleSparkMax4201(ModulePosition.BACK_LEFT,
              CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
              DriveConstants.kBackLeftDriveMotorReversed,
              DriveConstants.kBackLeftTurningMotorReversed,
              CanConstants.BACK_LEFT_MODULE_STEER_OFFSET),

          ModulePosition.BACK_RIGHT,
          new SwerveModuleSparkMax4201(
              ModulePosition.BACK_RIGHT,
              CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
              DriveConstants.kBackRightDriveMotorReversed,
              DriveConstants.kBackRightTurningMotorReversed,
              CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET)));
  // The gyro sensor

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      getHeadingRotation2d(),
      new Pose2d(),
      kSwerveKinematics,
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.025),
      VecBuilder.fill(0.1, 0.1, 0.1));

  private boolean showOnShuffleboard = true;

  private SimDouble m_simAngle;// navx sim

  public Map<ModulePosition, Translation2d> kModuleTranslations = DriveConstants.kModuleTranslations;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_gyro.reset();

    resetModuleEncoders();

    if (showOnShuffleboard)

      moduleInitShuffleboard();

    setIdleMode(true);

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }
    // info
    SmartDashboard.putNumber("maxradps", DriveConstants.kMaxRotationRadiansPerSecond);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    SmartDashboard.putNumber("X", getX());
    SmartDashboard.putNumber("Y", getY());

    SmartDashboard.putNumber("Yaw", getHeadingDegrees());

  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kSwerveKinematics.toChassisSpeeds(
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459
    
    double temp = chassisSpeed.omegaRadiansPerSecond * 1.1459155;
  
    temp += m_simAngle.get();

    m_simAngle.set(temp);

  }

  public void updateOdometry() {
    m_odometry.update(
        getHeadingRotation2d(),
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Translation2d modulePositionFromChassis = kModuleTranslations
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

  public SwerveModuleSparkMax4201 getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder((m_gyro.getAngle()), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
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

  // nove the robot from gamepad
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

    // info
    SmartDashboard.putNumber("CHSPDX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("CHSPDY", chassisSpeeds.vyMetersPerSecond);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
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

    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void resetModuleEncoders() {
    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.resetEncoders();
  }

  public void moduleInitShuffleboard() {
    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))

      module.initShuffleboard();
  }

  public static double wrapAngleDeg(double angle) {
    angle %= 360;
    angle = angle > 180 ? angle - 360 : angle;
    angle = angle < -180 ? angle + 360 : angle;
    return angle;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);

  }

  public Translation2d getTranslation() {
    return getPoseMeters().getTranslation();
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
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setIdleMode(boolean brake) {
    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules)) {
      module.setDriveBrakeMode(brake);
      module.setTurnBrakeMode(brake);
    }

  }

}
