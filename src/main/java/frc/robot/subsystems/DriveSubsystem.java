// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
//import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;
import frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

  private final HashMap<ModulePosition, SwerveModuleSparkMax4201> m_swerveModules = new HashMap<>(

      Map.of(

          ModulePosition.FRONT_LEFT,
          new SwerveModuleSparkMax4201(ModulePosition.FRONT_LEFT,
              CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
              DriveConstants.kRearLeftDriveEncoderReversed,
              DriveConstants.kRearLeftTurningEncoderReversed),

          ModulePosition.BACK_LEFT,
          new SwerveModuleSparkMax4201(ModulePosition.BACK_LEFT,
              CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
              CanConstants.BACK_LEFT_MODULE_STEER_ENCODER,
              // CanConstants.kRearLeftTurningEncoderPorts,
              DriveConstants.kRearLeftDriveEncoderReversed,
              DriveConstants.kRearLeftTurningEncoderReversed),

          ModulePosition.FRONT_RIGHT,
          new SwerveModuleSparkMax4201(
              ModulePosition.FRONT_RIGHT,
              CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
              // CanConstants.kFrontRightTurningEncoderPorts,
              DriveConstants.kFrontRightDriveEncoderReversed,
              DriveConstants.kFrontRightTurningEncoderReversed),

          ModulePosition.BACK_RIGHT,
          new SwerveModuleSparkMax4201(
              ModulePosition.BACK_RIGHT,
              CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
              CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
              // CanConstants.kRearRightTurningEncoderPorts,
              DriveConstants.kRearRightDriveEncoderReversed,
              DriveConstants.kRearRightTurningEncoderReversed)));

  // private final SwerveDrivePoseEstimator m_odometry1 = new
  // SwerveDrivePoseEstimator(
  // getHeading(),
  // new Pose2d(),
  // kSwerveKinematics,
  // VecBuilder.fill(0.1, 0.1, 0.1),
  // VecBuilder.fill(0.05),
  // VecBuilder.fill(0.1, 0.1, 0.1));

  private boolean showOnShuffleboard = true;

  private boolean showOnGlass = true;

  public final Field2d m_field;

  // The gyro sensor
  // private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kSwerveKinematics, m_gyro.getRotation2d());

  private int loopCtr;

  public Map<ModulePosition, Translation2d> kModuleTranslations = DriveConstants.kModuleTranslations;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_field = new Field2d();

    SmartDashboard.putData("Field", m_field);

    moduleMecsShow();

    moduleDataShow();

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
    m_field.setRobotPose(getPose());
    moduleMecs();
  }
  // if (showOnGlass) {
  // if (loopCtr == 0) {
  // m_frontLeft.updateGlass();
  // SmartDashboard.putNumberArray("datfl", m_frontLeft.data);
  // }

  // if (loopCtr == 1) {
  // m_frontRight.updateGlass();
  // SmartDashboard.putNumberArray("datfr", m_frontRight.data);
  // }
  // if (loopCtr == 2) {
  // m_rearLeft.updateGlass();
  // SmartDashboard.putNumberArray("datrl", m_rearLeft.data);
  // }
  // if (loopCtr == 3) {
  // m_rearRight.updateGlass();
  // SmartDashboard.putNumberArray("datrr", m_rearRight.data);
  // }

  // loopCtr++;

  // if (loopCtr == 4)
  // loopCtr = 0;
  // }

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

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public SwerveModuleSparkMax4201 getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(m_gyro.getYaw(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
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
  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
    throttle *= DriveConstants.kMaxSpeedMetersPerSecond;
    strafe *= DriveConstants.kMaxSpeedMetersPerSecond;
    rotation *= DriveConstants.kaVoltSecondsSquaredPerMeter;

    ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            throttle, strafe, rotation, getHeadingRotation2d())
        : new ChassisSpeeds(throttle, strafe, rotation);

    Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
        .of(kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), DriveConstants.kMaxSpeedMetersPerSecond);

    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()));
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
      module.setDesiredState(states[module.getModulePosition().ordinal()]);
  }

  public void moduleMecs() {
    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.updateGlass();
  }

  public void moduleMecsShow() {
    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.showMec();
  }

  public void moduleDataShow() {
    for (SwerveModuleSparkMax4201 module : ModuleMap.orderedValuesList(m_swerveModules))
      module.showData();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_frontLeft.resetEncoders();
    // m_rearLeft.resetEncoders();
    // m_frontRight.resetEncoders();
    // m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // public Rotation2d getHeading() {
  // return m_gyro.getRotation2d();
  // }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
