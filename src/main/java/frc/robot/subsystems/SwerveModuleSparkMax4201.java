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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleSparkMax4201 extends SubsystemBase {
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveSMPIDController;
  private final SparkMaxPIDController m_turnSMPIDController;

  private final CTRECanCoder m_canEncoder;
  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  ModulePosition m_modulePosition;
  private double startTime;
  int m_moduleNumber;
  private int lpctr;
  public String layout;
  // Using a TrapezoidProfile PIDController to allow for smooth turning
  public ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          Units.radiansToDegrees(AutoConstants.kMaxRotationRadiansPerSecond),
          AutoConstants.kMaxRotationRadiansPerSecondSquared));

  Pose2d m_pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  private double angle;
  private double actualAngleDegrees;

  private double actualDriveMeters;
  private double actualDriveRate;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param driveEncoderChannels   The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
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

    m_driveSMPIDController = m_driveMotor.getPIDController();

    m_driveSMPIDController.setP(10);

    m_turningEncoder = m_turningMotor.getEncoder();

    m_turnSMPIDController = m_turningMotor.getPIDController();

    m_turnSMPIDController.setP(50);

    m_canEncoder = new CTRECanCoder(cancoderCanChannel);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    SmartDashboard.putNumber("DriveGearRatio", ModuleConstants.mk4iL1DriveGearRatio);

    SmartDashboard.putNumber("DriveMetersPerEncReav", ModuleConstants.kDriveMetersPerEncRev);
    SmartDashboard.putNumber("TurnGearRatio", ModuleConstants.mk4iL1TurnGearRatio);

    SmartDashboard.putNumber("TurnDegreesPerPulse", ModuleConstants.kTurningDegreesPerEncRev);

    SmartDashboard.putNumber("Max Ang Radspersec", AutoConstants.kMaxRotationRadiansPerSecond);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-180, 180);

    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();

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
    actualDriveRate = (m_driveEncoder.getVelocity() * ModuleConstants.kDriveMetersPerEncRev) / 60;

    double driveOutput = m_drivePIDController.calculate(actualDriveRate, state.speedMetersPerSecond);

    {

      SmartDashboard.putNumber("DSSMS" + String.valueOf(m_moduleNumber), desiredState.speedMetersPerSecond);

      if (RobotBase.isReal()) {
        if (isOpenLoop)
          driveOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

        m_driveMotor.set(driveOutput);

      } else {

        m_driveSMPIDController.setReference(driveOutput * 12, ControlType.kVelocity);

      }
      // turn motor code
      angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
          ? m_lastAngle
          : desiredState.angle
              .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
      m_lastAngle = angle;
      // Jittering.
      // angle = 120;
      // Calculate the turning motor output from the turning PID controller.
      actualAngleDegrees = m_turningEncoder.getPosition() * ModuleConstants.kTurningDegreesPerEncRev;
      actualAngleDegrees = wrapAngleDeg(actualAngleDegrees);
    }

    double turnOutput = m_turningPIDController.calculate(actualAngleDegrees,
        angle);

    if (RobotBase.isReal()) {
      turnOutput = turnOutput / AutoConstants.kMaxRotationRadiansPerSecond;
      if (turnOutput > 1)
        turnOutput = 1;
      if (turnOutput < -1)
        turnOutput = -1;
      m_turningMotor.set(turnOutput);
    } else {
      m_turnSMPIDController.setReference(turnOutput, ControlType.kVelocity);
    }
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

}
