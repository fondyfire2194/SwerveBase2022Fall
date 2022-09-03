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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Pref;
import frc.robot.utils.DriveUtils;

public class SwerveModuleSparkMax4201 extends SubsystemBase {
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

  Pose2d m_pose;

  double testAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  private double angle;
  private double actualAngleDegrees;
  private double angleError;

  double m_turningEncoderOffset;

  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;

  int SIM_SLOT = 0;

  private int tuneOn;
  private double pidout;

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

      m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMetersPerEncRev);

      m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev / 60);

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

    m_turnSMController.setP(Pref.getPref("SwerveTurnPoskP"), VEL_SLOT);
    m_turnSMController.setI(Pref.getPref("SwerveTurnPoskI"), VEL_SLOT);
    m_turnSMController.setD(Pref.getPref("SwerveTurnPoskD"), VEL_SLOT);
    m_turnSMController.setIZone(Pref.getPref("SwerveTurnPoskIz"), VEL_SLOT);

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
      return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(Units.radiansToDegrees(angle)));
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

    state = DriveUtils.optimize(desiredState, getHeadingRotation2d());

    // state = SwerveModuleState.optimize(desiredState, new
    // Rotation2d(actualAngleDegrees));

    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
        ? m_lastAngle
        : state.angle.getDegrees();

    m_lastAngle = angle;

    if (RobotBase.isReal())

      actualAngleDegrees = m_turningEncoder.getPosition();

    else
      actualAngleDegrees = angle;

    if (RobotBase.isReal()) {

      m_driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kFreeMetersPerSecond);

      m_turnSMController.setReference(angle, ControlType.kPosition, POS_SLOT);

    }

    if (RobotBase.isSimulation()) {

      m_driveVelController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, SIM_SLOT);

      // no simulation for angle - angle command is returned directly to drive
      // subsystem as actual angle in 2 places - getState() and getHeading

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

    drLayout.addNumber("Drive Setpoint MPS " + String.valueOf(m_moduleNumber), () -> getState().speedMetersPerSecond);

    drLayout.addNumber("Drive Speed MPS " + String.valueOf(m_moduleNumber), () -> m_driveEncoder.getVelocity());

    drLayout.addNumber("Drive Position " + String.valueOf(m_moduleNumber), () -> m_driveEncoder.getPosition());

    turnLayout = m_modulePosition.toString() + " Turn";

    ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
        .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 2)
        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

    tuLayout.addNumber("Turn Setpoint Degrees " + String.valueOf(m_moduleNumber), () -> angle);

    tuLayout.addNumber("Turn Angle " + String.valueOf(m_moduleNumber),
        () -> m_turningEncoder.getPosition());

    tuLayout.addNumber("TurnAngleOut" + String.valueOf(m_moduleNumber), () -> m_turningMotor.getAppliedOutput());

    tuLayout.addNumber("AngERR" + String.valueOf(m_moduleNumber), () -> angleError);

    tuLayout.addNumber("AngPID" + String.valueOf(m_moduleNumber), () -> pidout);
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
    
      return angle;

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
    m_turningEncoder.setPosition(angle);
  }

}
