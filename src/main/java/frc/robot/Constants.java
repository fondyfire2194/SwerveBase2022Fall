// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CanConstants {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_CANCODER = 6;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 231.5;// -Math.toRadians(0.0);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_CANCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 317;// -Math.toRadians(-42);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_CANCODER = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 181.1;// -Math.toRadians(0.0);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
    public static final int BACK_RIGHT_MODULE_STEER_CANCODER = 15;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 253.7;// -Math.toRadians(-105);

  }

  public static final class DriveConstants {

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    public static final double kTrackWidth = Units.inchesToMeters(22);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);

    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

    public static final Map<ModulePosition, Translation2d> kModuleTranslations = Map.of(
        ModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        ModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        ModulePosition.BACK_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        ModulePosition.BACK_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;

    public static final double kMaxRotationRadiansPerSecond = Math.PI;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI;

    public static final double kP_X = 0.2;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.2;
    public static final double kD_Y = 0;
    public static final double kP_Theta = 8;
    public static final double kD_Theta = 0;
    public static double kTranslationSlew = 1.55;
    public static double kRotationSlew = 3.00;
    public static double kControllerDeadband = .05;
    public static double kControllerRotDeadband = .1;

    public static double kVoltCompensation = 12.6;

    // public static final double kMaxRotationRadiansPerSecond =
    // Math.hypot(DriveConstants.kTrackWidth / 2.0,
    // DriveConstants.kWheelBase / 2.0);

    // public static final double MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED = 2 *
    // Math.PI;

  }

  public static final class ModuleConstants {

    // ModuleConfiguration MK4I_L1
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14 .122807

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static final double kDriveMetersPerEncRev =

        (kWheelDiameterMeters * Math.PI) / mk4iL1DriveGearRatio;// 0.039198257811106

    public static final double kDriveEncRPMperMPS = kDriveMetersPerEncRev / 60;// 0.000653304296852

    public static double kEncoderRevsPerMeter = 1 / kDriveMetersPerEncRev;// 25.511337897182322

    public static double kFreeMetersPerSecond = 5600 * kDriveEncRPMperMPS;// 3.6

    public static final double kTurningDegreesPerEncRev =

        360 / mk4iL1TurnGearRatio;

    // max turn speed = (5400/ 21.43) revs per min 240 revs per min 4250 deg per
    // min
    public static final double kPModuleTurningController = .025;

    public static final double kPModuleDriveController = .2;

    // use sysid on robot
    public static double ksVolts = .055;
    public static double kvVoltSecondsPerMeter = .2;
    public static double kaVoltSecondsSquaredPerMeter = .02;

    public static double kPModuleTurnController;

    public static double kSMmaxAccel = 90;// deg per sec per sec

    public static double maxVel = 90; // deg per sec

    public static double allowedErr = .05;// deg

    // sysid on module?
    public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;
    // sysid on module?
    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3

    public static double kMaxModuleAngularSpeedDegPerSec = 90;

    public static final double kMaxModuleAngularAccelerationDegreesPerSecondSquared = 90;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;

  }

  public static final class TrapezoidConstants {

    public static final double kMaxSpeedMetersPerSecond = 3;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // public static final double kMaxAngularSpeedDegreesPerSecond = 800;

    // public static final double kMaxAngularSpeedDegreesPerSecondSquared =2000;
    public static final double kMaxRotationRadiansPerSecond = Math.PI;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(

        kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

  }

  public static class VisionConstants {

    /**
     * Physical location of the camera on the robot, relative to the center of the
     * robot.
     */
    public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(new Translation2d(-0.3425, 0.0),
        new Rotation2d(0.0));

    // 15"up and 15"forward

    private static double camHeight = Units.inchesToMeters(15);

    public static double camXFromCenter = -Units.inchesToMeters(15);

    public static final Transform3d CAMERA_TO_ROBOT_3D = new Transform3d(new Translation3d(camXFromCenter, 0.0, camHeight),
        new Rotation3d());
  }

public static final int LED_CONTROLLER_PORT = 1;
}