package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class AngleUtils {
  

  public static CANCoderConfiguration generateCanCoderConfig() {
    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    return sensorConfig;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
