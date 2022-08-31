// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.CTREPCMJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TrapezoidConstants;

public class SwerveModuleSparkMax4201 extends SubsystemBase {
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveVelController;

  private final SparkMaxPIDController m_turnVelController;

  private final CTRECanCoder m_turnCANcoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  ModulePosition m_modulePosition;// enum with test module names

  int m_moduleNumber;

  String layout;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  public ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          Units.radiansToDegrees(TrapezoidConstants.kMaxRotationRadiansPerSecond),
          TrapezoidConstants.kMaxRotationRadiansPerSecondSquared));

  Pose2d m_pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  private double angle;
  private double actualAngleDegrees;

  private double actualDriveRate;

  double m_turningEncoderOffset;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor.
   * @param turningMotorChannel     The channel of the turning motor.
   * @param driveEncoderChannels    The channels of the drive encoder.
   * @param turningCANCoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed    Whether the drive encoder is reversed.
   * @param turningEncoderReversed  Whether the turning encoder is reversed.
   * @param turningEncoderOffset
   */
  public SwerveModuleSparkMax4201(
      ModulePosition modulePosition,
      int driveMotorCanChannel,
      int turningMotorCanChannel,
      int cancoderCanChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double turningEncoderOffset) {

    m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);

    m_turningMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);

    // absolute encoder used to establish known wheel position on start position
    m_turnCANcoder = new CTRECanCoder(cancoderCanChannel);

    m_turningEncoderOffset = turningEncoderOffset;

    m_driveMotor.setInverted(driveMotorReversed);

    m_turningMotor.setInverted(turningMotorReversed);

    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveVelController = m_driveMotor.getPIDController();

    if (RobotBase.isReal()) {

      m_driveVelController.setP(1);

      m_driveVelController.setD(0);

      m_driveVelController.setI(0);

      m_driveVelController.setIZone(1);

    }

    else {

      m_driveVelController.setP(100);
    }

    m_turningEncoder = m_turningMotor.getEncoder();

    m_turnVelController = m_turningMotor.getPIDController();

    if (RobotBase.isReal()) {

      m_turnVelController.setP(1);

      m_turnVelController.setD(0);

      m_turnVelController.setI(0);

      m_turnVelController.setIZone(1);

    }

    else {

      m_turnVelController.setP(200);

      m_turningPIDController.setP(1);
    }

    // info
    SmartDashboard.putNumber("DriveGearRatio", ModuleConstants.mk4iL1DriveGearRatio);

    SmartDashboard.putNumber("DriveMetersPerEncReav", ModuleConstants.kDriveMetersPerEncRev);

    SmartDashboard.putNumber("TurnGearRatio", ModuleConstants.mk4iL1TurnGearRatio);

    SmartDashboard.putNumber("TurnDegreesPerEncRev", ModuleConstants.kTurningDegreesPerEncRev);

    SmartDashboard.putNumber("Max Ang Radspersec", TrapezoidConstants.kMaxRotationRadiansPerSecond);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-180, 180);

    m_modulePosition = modulePosition;

    m_moduleNumber = m_modulePosition.ordinal();// gets module enum index

    if (RobotBase.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));

      REVPhysicsSim.getInstance().addSparkMax(m_turningMotor, DCMotor.getNEO(1));

      m_turningPIDController.setP(100);

    }
    resetEncoders();
  }

  @Override
  public void simulationPeriodic() {

    REVPhysicsSim.getInstance().run();

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {

    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(actualAngleDegrees));
  }

  public ModulePosition getModulePosition() {
    return m_modulePosition;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Optimize the reference state to avoid spinning further than 90 degrees

    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(actualAngleDegrees));

    // Calculate the drive output from the drive PID controller.
    actualDriveRate = (m_driveEncoder.getVelocity() / 60 * ModuleConstants.kDriveMetersPerEncRev) / 60;

    double driveOutput = m_drivePIDController.calculate(actualDriveRate, state.speedMetersPerSecond);

    m_driveVelController.setReference(driveOutput * 11, ControlType.kVelocity);

    
    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
        ? m_lastAngle
        : desiredState.angle
            .getDegrees();
    m_lastAngle = angle;

    SmartDashboard.putNumber("TurnAngle" + String.valueOf(m_moduleNumber), desiredState.angle.getDegrees());

    SmartDashboard.putNumber("DESSPEED" + String.valueOf(m_moduleNumber), desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("ANGPERR" + String.valueOf(m_moduleNumber), m_turningPIDController.getPositionError());

    // Calculate the turning motor output from the turning PID controller.
    // encoder counts are in decimal fraction revolutions
    // need to convert them to degrees for use with command angle
    actualAngleDegrees = m_turningEncoder.getPosition() * ModuleConstants.kTurningDegreesPerEncRev;
    // keeps angle within +-180 degrees
    actualAngleDegrees = wrapAngleDeg(actualAngleDegrees);
    // get the angle error
    double turnOutput = m_turningPIDController.calculate(actualAngleDegrees,
        angle);

    // feed the error in degrees per second to the SparkMax velocity loop
    // different P values are used for simulation
    m_turnVelController.setReference(turnOutput, ControlType.kVelocity);

  }

  public void initShuffleboard() {
    layout = m_modulePosition.toString();
    ShuffleboardLayout xLayout = Shuffleboard.getTab("Drivetrain")
        .getLayout(layout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
        .withSize(2, 5).withProperties(Map.of("Label position", "TOP"));

    xLayout.addNumber("Turn Setpoint Degrees " + String.valueOf(m_moduleNumber), () -> angle);
    xLayout.addNumber("Turn Angle Encoder " + String.valueOf(m_moduleNumber),
        () -> m_turningEncoder.getPosition());
    xLayout.addNumber("TurnAngleDegrees", () -> actualAngleDegrees);
    xLayout.addNumber("TurnAngleVolts" + String.valueOf(m_moduleNumber), () -> m_turningMotor.getAppliedOutput());
    xLayout.addNumber("Drive Setpoint MPS " + String.valueOf(m_moduleNumber), () -> getState().speedMetersPerSecond);
    xLayout.addNumber("Drive Speed MPS " + String.valueOf(m_moduleNumber), () -> m_driveEncoder.getVelocity());

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    // m_canEncoder.

  }

  public double getHeadingDegrees() {

    return actualAngleDegrees;

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public static double wrapAngleDeg(double angle) {
    angle %= 360;
    angle = angle > 180 ? angle - 360 : angle;
    angle = angle < -180 ? angle + 360 : angle;
    return angle;
  }

  public static double limitMotorCmd(double motorCmdIn) {
    return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
  }

  public void setDriveBrakeMode(boolean on) {
    if (on)
      m_driveMotor.setIdleMode(IdleMode.kBrake);
    else
      m_driveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean on) {
    if (on)
      m_turningMotor.setIdleMode(IdleMode.kBrake);
    else
      m_turningMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetAngleToAbsolute() {
    double angle = m_turnCANcoder.getAbsolutePosition() - m_turningEncoderOffset;
    m_turningEncoder.setPosition(angle / ModuleConstants.kTurningDegreesPerEncRev);
  }

}
