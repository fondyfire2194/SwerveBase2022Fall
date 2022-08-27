// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {

  private boolean showOnShuffleboard = true;

  private boolean showOnGlass = true;

  public final Field2d m_field;

  // Robot swerve modules
  public final SwerveModuleSparkMax m_frontLeft = new SwerveModuleSparkMax(
      CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
      // CanConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModuleSparkMax m_rearLeft = new SwerveModuleSparkMax(
      CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_ENCODER,
      // CanConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModuleSparkMax m_frontRight = new SwerveModuleSparkMax(
      CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
      // CanConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModuleSparkMax m_rearRight = new SwerveModuleSparkMax(
      CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
      // CanConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  // private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  private int loopCtr;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_field = new Field2d();

    SmartDashboard.putData("Field", m_field);

    if (showOnGlass) {
      SmartDashboard.putData("frontleft", m_frontLeft.mech);
      SmartDashboard.putData("frontright", m_frontRight.mech);
      SmartDashboard.putData("rearleft", m_rearLeft.mech);
      SmartDashboard.putData("rearright", m_rearRight.mech);

    }

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());

    m_field.setRobotPose(getPose());

    if (showOnGlass) {
      if (loopCtr == 0) {
        m_frontLeft.updateGlass();
        SmartDashboard.putNumberArray("datfl", m_frontLeft.data);
      }

      if (loopCtr == 1) {
        m_frontRight.updateGlass();
        SmartDashboard.putNumberArray("datfr", m_frontRight.data);
      }
      if (loopCtr == 2) {
        m_rearLeft.updateGlass();
        SmartDashboard.putNumberArray("datrl", m_rearLeft.data);
      }
      if (loopCtr == 3) {
        m_rearRight.updateGlass();
        SmartDashboard.putNumberArray("datrr", m_rearRight.data);
      }

      loopCtr++;

      if (loopCtr == 4)
        loopCtr = 0;
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
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
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
