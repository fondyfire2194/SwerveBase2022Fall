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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Pref;
import frc.robot.utils.AngleUtils;

public class SwerveModuleSparkMax extends SubsystemBase {
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveVelController;

  private final SparkMaxPIDController m_turnSMController;

  private final CTRECanCoder m_turnCANcoder;

  ModulePosition m_modulePosition;// enum with test module names;

  SwerveModuleState state;

  int m_moduleNumber;

  String driveLayout;

  String turnLayout;

  String canCoderLayout;

  Pose2d m_pose;

  double testAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  private double angle;

  double m_turningEncoderOffset;

  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;
  private final int SIM_SLOT = 2;

  private int tuneOn;
  private double actualAngleDegrees;

  private double angleDifference;
  private double angleIncrementPer20ms;
  private double tolDegPerSec = .05;
  private double toleranceDeg = .25;

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
  public SwerveModuleSparkMax(
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

    m_turningMotor.setSmartCurrentLimit(20);

    m_driveMotor.setSmartCurrentLimit(20);

    // absolute encoder used to establish known wheel position on start position
    m_turnCANcoder = new CTRECanCoder(cancoderCanChannel);
    m_turnCANcoder.configFactoryDefault();
    m_turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());

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

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);

    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncRPMperMPS);

    m_driveVelController = m_driveMotor.getPIDController();

    if (RobotBase.isReal()) {

      m_driveVelController.setP(.01, VEL_SLOT);

      m_driveVelController.setD(0, VEL_SLOT);

      m_driveVelController.setI(0, VEL_SLOT);

      m_driveVelController.setIZone(1, VEL_SLOT);

    }

    else {

      m_driveVelController.setP(1, SIM_SLOT);

    }

    m_turningEncoder = m_turningMotor.getEncoder();

    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);

    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

    m_turnSMController = m_turningMotor.getPIDController();

    m_turnSMController.setP(Pref.getPref("SwerveTurnPoskP"), POS_SLOT);
    m_turnSMController.setI(Pref.getPref("SwerveTurnPoskI"), POS_SLOT);
    m_turnSMController.setD(Pref.getPref("SwerveTurnPoskD"), POS_SLOT);
    m_turnSMController.setIZone(Pref.getPref("SwerveTurnPoskIz"), POS_SLOT);

    m_modulePosition = modulePosition;

    m_moduleNumber = m_modulePosition.ordinal();// gets module enum index

    if (RobotBase.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));

    }
    resetEncoders();

  }

  @Override
  public void periodic() {
    if (Pref.getPref("SwerveTune") == 1 && tuneOn == 0) {
      tuneGains();
      tuneOn = 1;
    }
    if (tuneOn == 1) {
      tuneOn = (int) Pref.getPref("SwerveTune");
    }
    if (m_turnCANcoder.getFaulted()) {
      // SmartDashboard.putStringArray("CanCoderFault"
      // + m_modulePosition.toString(), m_turnCANcoder.getFaults());
      SmartDashboard.putStringArray("CanCoderStickyFault"
          + m_modulePosition.toString(), m_turnCANcoder.getStickyFaults());
    }
  }

  public void tuneGains() {
    m_turnSMController.setP(Pref.getPref("SwerveTurnPoskP"), POS_SLOT);
    m_turnSMController.setI(Pref.getPref("SwerveTurnPoskI"), POS_SLOT);
    m_turnSMController.setD(Pref.getPref("SwerveTurnPoskD"), POS_SLOT);
    m_turnSMController.setIZone(Pref.getPref("SwerveTurnPoskIz"), POS_SLOT);
  }

  @Override
  public void simulationPeriodic() {

    REVPhysicsSim.getInstance().run();

  }

  public SwerveModuleState getState() {
    if (RobotBase.isReal())
      return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getHeadingDegrees()));
    else
      return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(Units.degreesToRadians(angle)));
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

    state = AngleUtils.optimize(desiredState, getHeadingRotation2d());

    // state = SwerveModuleState.optimize(desiredState, new
    // Rotation2d(actualAngleDegrees));

    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
        ? m_lastAngle
        : state.angle.getDegrees();

    m_lastAngle = angle;

    if (RobotBase.isReal()) {

      actualAngleDegrees = m_turningEncoder.getPosition();

      m_turnSMController.setReference(angle, ControlType.kPosition, POS_SLOT);

      if (isOpenLoop)

        m_driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kFreeMetersPerSecond);

      else

        m_driveVelController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, VEL_SLOT);

    }

    if (RobotBase.isSimulation()) {

      m_driveVelController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, SIM_SLOT);

      // no simulation for angle - angle command is returned directly to drive
      // subsystem as actual angle in 2 places - getState() and getHeading

      if (angle != actualAngleDegrees && angleIncrementPer20ms == 0) {
        angleDifference = angle - actualAngleDegrees;
        angleIncrementPer20ms = angleDifference / 20;// 10*20ms = .2 sec move time
      }

      if (angleIncrementPer20ms != 0) {
        actualAngleDegrees += angleIncrementPer20ms;
        if ((Math.abs(angle - actualAngleDegrees)) < .01) {
          actualAngleDegrees = angle;
          angleIncrementPer20ms = 0;
        }
      }
    }

  }

  public static double limitMotorCmd(double motorCmdIn) {
    return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
  }

  public void initShuffleboard() {
    driveLayout = m_modulePosition.toString() + " Drive";
    ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
        .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
        .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

    drLayout.addNumber("Drive Speed MPS " + m_modulePosition.toString(), () -> m_driveEncoder.getVelocity());

    drLayout.addNumber("Drive Position " + m_modulePosition.toString(), () -> m_driveEncoder.getPosition());

    drLayout.addNumber("App Output " + m_modulePosition.toString(), () -> m_driveMotor.getAppliedOutput());

    turnLayout = m_modulePosition.toString() + " Turn";

    ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
        .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

    tuLayout.addNumber("Turn Setpoint Deg " + m_modulePosition.toString(), () -> angle);

    tuLayout.addNumber("Turn Enc Pos " + m_modulePosition.toString(),
        () -> m_turningEncoder.getPosition() % 360);

    tuLayout.addNumber("Act Ang Deg " + m_modulePosition.toString(),
        () -> actualAngleDegrees);

    tuLayout.addNumber("TurnAngleOut" + m_modulePosition.toString(), () -> m_turningMotor.getAppliedOutput());

    tuLayout.addNumber("Position" + m_modulePosition.toString(), () -> m_turnCANcoder.getMyPosition());

    tuLayout.addNumber("Abs Offset" + m_modulePosition.toString(), () -> m_turningEncoderOffset);

    tuLayout.addString("Fault" + m_modulePosition.toString(), () -> m_turnCANcoder.getFaulted() ? "TRUE" : "FALSE");

  }

  public void initShuffleboardCanCoder() {

    canCoderLayout = m_modulePosition.toString() + " CanCoder";

    ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
        .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

    coderLayout.addNumber("Position" + m_modulePosition.toString(), () -> m_turnCANcoder.getMyPosition());
    coderLayout.addNumber("Abs Position" + m_modulePosition.toString(), () -> m_turnCANcoder.getAbsolutePosition());
    coderLayout.addNumber("Velocity" + m_modulePosition.toString(), () -> m_turnCANcoder.getVelValue());
    coderLayout.addString(" MagField " + m_modulePosition.toString(),
        () -> m_turnCANcoder.getMagnetFieldStrength().toString());
    coderLayout.addBoolean("Has Fault" + m_modulePosition.toString(),
        () -> m_turnCANcoder.getFaulted());
    coderLayout.addNumber("Battery Volts" + m_modulePosition.toString(),
        () -> m_turnCANcoder.getBatValue());
    coderLayout.addNumber("Bus Volts" + m_modulePosition.toString(),
        () -> m_turnCANcoder.getBusVoltage());

    coderLayout.addNumber("Abs Offset" + m_modulePosition.toString(), () -> m_turningEncoderOffset);

  }

  public void initTuningShuffleboard() {

    ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
        .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

    tuLayout.addNumber("Turn Setpoint Deg " + m_modulePosition.toString(), () -> angle);

    tuLayout.addNumber("Turn Enc Pos " + m_modulePosition.toString(),
        () -> m_turningEncoder.getPosition());

    tuLayout.addNumber("Act Ang Deg " + m_modulePosition.toString(),
        () -> actualAngleDegrees);

    tuLayout.addNumber("TurnAngleOut" + m_modulePosition.toString(), () -> m_turningMotor.getAppliedOutput());

    tuLayout.addNumber("Position" + m_modulePosition.toString(), () -> m_turnCANcoder.getMyPosition());

    tuLayout.addNumber("Abs Offset" + m_modulePosition.toString(), () -> m_turningEncoderOffset);

    tuLayout.addString("Fault" + m_modulePosition.toString(), () -> m_turnCANcoder.getFaulted() ? "TRUE" : "FALSE");

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    // m_canEncoder.

  }

  public double getHeadingDegrees() {

    if (RobotBase.isReal())

      return m_turningEncoder.getPosition();

    else

      return actualAngleDegrees;

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
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
    m_turningEncoder.setPosition(-angle);
  }

  public double getTurnAngle() {
    return m_turningEncoder.getPosition();
  }

  public void turnMotorMove(double speed) {
    m_turningMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void positionTurn(double angle) {

    m_turnSMController.setReference(angle, ControlType.kPosition, POS_SLOT);
  }

  public void driveMotorMove(double speed) {
    m_driveMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public boolean turnInPosition(double targetAngle) {
    return Math.abs(targetAngle - getTurnAngle()) < toleranceDeg;
  }

  public boolean turnIsStopped() {
    return Math.abs(m_turningEncoder.getVelocity()) < tolDegPerSec;
  }

}
