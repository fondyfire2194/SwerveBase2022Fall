// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.Pref;

public class SwerveModuleSparkMax4201 extends SubsystemBase {
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveVelController;

  private final SparkMaxPIDController m_turnPosController;

  private final CTRECanCoder m_turnCANcoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final PIDController m_turnPIDController = new PIDController(1, 0, 0);

  ModulePosition m_modulePosition;// enum with test module names

  int m_moduleNumber;

  String layout;

  Pose2d m_pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  private double angle;
  private double actualAngleDegrees;

  private double optimumAngle;

  private double actualDriveRate;

  double m_turningEncoderOffset;

  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;

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

    m_turningMotor.restoreFactoryDefaults();

    m_driveMotor.restoreFactoryDefaults();

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

    m_driveEncoder.setPositionConversionFactor(1);
    m_driveEncoder.setVelocityConversionFactor(1 / 60);

    m_driveVelController = m_driveMotor.getPIDController();

    if (RobotBase.isReal()) {

      m_driveVelController.setP(.01);

      m_driveVelController.setD(0);

      m_driveVelController.setI(0);

      m_driveVelController.setIZone(1);

    }

    else {

      m_driveVelController.setP(1);

    }

    m_turningEncoder = m_turningMotor.getEncoder();

    m_turningEncoder.setPositionConversionFactor(1);

    m_turningEncoder.setVelocityConversionFactor(1 / 60);

    m_turnPosController = m_turningMotor.getPIDController();

    if (RobotBase.isReal()) {
      m_turnPosController.setP(Pref.getPref("SwerveTurnPoskP"), POS_SLOT);
      m_turnPosController.setI(Pref.getPref("SwerveTurnPoskI"), POS_SLOT);
      m_turnPosController.setD(Pref.getPref("SwerveTurnPosKD"), POS_SLOT);
      m_turnPosController.setIZone(Pref.getPref("SwerveTurnPosKIz"), POS_SLOT);
    }

    else {

      m_turnPosController.setP(1, VEL_SLOT);

      m_turnPIDController.setP(1);

    }
    m_turnPIDController.enableContinuousInput(-180, 180);
    // info
    SmartDashboard.putNumber("DriveGearRatio", ModuleConstants.mk4iL1DriveGearRatio);

    SmartDashboard.putNumber("DriveMetersPerEncRev", ModuleConstants.kDriveMetersPerEncRev);

    SmartDashboard.putNumber("TurnGearRatio", ModuleConstants.mk4iL1TurnGearRatio);

    SmartDashboard.putNumber("TurnDegreesPerEncRev", ModuleConstants.kTurningDegreesPerEncRev);

    SmartDashboard.putNumber("Max Ang Radspersec", TrapezoidConstants.kMaxRotationRadiansPerSecond);

    m_modulePosition = modulePosition;

    m_moduleNumber = m_modulePosition.ordinal();// gets module enum index

    if (RobotBase.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));

      REVPhysicsSim.getInstance().addSparkMax(m_turningMotor, DCMotor.getNEO(1));

    }
    resetEncoders();
  }

  @Override
  public void periodic() {
    if (Pref.getPref("dRTune") == 1) {
      m_turnPosController.setP(Pref.getPref("SwerveTurnPoskP"));
      m_turnPosController.setI(Pref.getPref("SwerveTurnPoskI"));
      m_turnPosController.setD(Pref.getPref("SwerveTurnPosKD"));
      m_turnPosController.setIZone(Pref.getPref("SwerveTurnPosKIz"));

    }

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

    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(actualAngleDegrees));

    // Calculate the drive output from the drive PID controller in mps.
    // encoder rate is in rpm so need to divide it by 60 for rps then convert revs
    // to meters

    actualDriveRate = m_driveEncoder.getVelocity() * ModuleConstants.kDriveEncRPSToMPS;

    double driveOutput = m_drivePIDController.calculate(actualDriveRate, state.speedMetersPerSecond);

    double drff80 = state.speedMetersPerSecond * .8;

    if (RobotBase.isSimulation())

      m_driveVelController.setReference(drff80 + driveOutput, ControlType.kVelocity);

    else

      m_driveMotor.set(state.speedMetersPerSecond / 3.5);

       // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
        ? m_lastAngle
        : state.angle
            .getDegrees();
    m_lastAngle = angle;

    // Calculate the turning motor output from the turning PID controller.
    // encoder counts are in decimal fraction revolutions
    // need to convert them to degrees for use with command angle
    actualAngleDegrees = m_turningEncoder.getPosition() * ModuleConstants.kTurningDegreesPerEncRev;
    // keeps angle within +-180 degrees
    actualAngleDegrees = wrapAngleDeg(actualAngleDegrees);

    double angleError = m_turnPIDController.calculate(actualAngleDegrees, angle);

    
    SmartDashboard.putNumber("Angle" + String.valueOf(m_moduleNumber), state.angle.getDegrees());
    SmartDashboard.putNumber("OptAngle" + String.valueOf(m_moduleNumber), optimumAngle);
    SmartDashboard.putNumber("ActAngle" + String.valueOf(m_moduleNumber), actualAngleDegrees);
    SmartDashboard.putNumber("ERRAngle" + String.valueOf(m_moduleNumber), optimumAngle - actualAngleDegrees);

    if (RobotBase.isReal()) {
      // feed the error in degrees per second to the SparkMax velocity loop
      m_turnPosController.setReference(angleError, ControlType.kPosition,
          POS_SLOT);
    }
    // need to use veocity controller as position doesn't work in simulation
    // So the position error needs to drive a vel controller
    // max err is 180 degrees max velocity is
    else {
      m_turnPosController.setReference((optimumAngle - actualAngleDegrees) / 5,
          ControlType.kVelocity, VEL_SLOT);
      // m_turnPosController.setReference(0, ControlType.kVelocity, VEL_SLOT);
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
    xLayout.addNumber("TurnAngleDegrees" + String.valueOf(m_moduleNumber),
        () -> actualAngleDegrees = wrapAngleDeg(
            m_turningEncoder.getPosition() * ModuleConstants.kTurningDegreesPerEncRev));
    xLayout.addNumber("TurnAngleOut" + String.valueOf(m_moduleNumber), () -> m_turningMotor.getAppliedOutput());
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

  private double getOptAngle(double measurement, double setpoint) {

    return MathUtil.inputModulus(setpoint - measurement, -180, 180);
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
