// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import frc.robot.commands.swerve.*;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.Pref;
import frc.robot.Robot;

public class SwerveModuleSparkMax4201 extends SubsystemBase {
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveVelController;

  private final SparkMaxPIDController m_turnVelCotroller;

  private final CTRECanCoder m_turnCANcoder;

  private final PIDController m_turnPIDController = new PIDController(1, 0.00001, 0);

  ModulePosition m_modulePosition;// enum with test module names;

  SwerveModuleState state;

  int m_moduleNumber;

  String driveLayout;

  String turnLayout;

  Pose2d m_pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  private double angle;
  private double actualAngleDegrees;

  private int simRamp = 500;// 500*20 ms = 1 sexc
  private int simAngleCounter;

  private double simAngle;

  double m_turningEncoderOffset;

  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;

  int SIM_SLOT = 0;

  private double optimumAngleDegrees;
  private boolean angleSimInitted;
  private boolean angleChange;
  private int tuneOn;

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

    m_turnVelCotroller = m_turningMotor.getPIDController();

    m_turnVelCotroller.setP(Pref.getPref("SwerveTurnPoskP"), POS_SLOT);
    m_turnVelCotroller.setI(Pref.getPref("SwerveTurnPoskI"), POS_SLOT);
    m_turnVelCotroller.setD(Pref.getPref("SwerveTurnPosKD"), POS_SLOT);
    m_turnVelCotroller.setIZone(Pref.getPref("SwerveTurnPosKIz"), POS_SLOT);

    m_turnVelCotroller.setP(2, VEL_SLOT);

    m_turnPIDController.setP(200);

    m_turnPIDController.enableContinuousInput(-180, 180);
    // info

    m_modulePosition = modulePosition;

    m_moduleNumber = m_modulePosition.ordinal();// gets module enum index

    if (RobotBase.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));

      REVPhysicsSim.getInstance().addSparkMax(m_turningMotor, DCMotor.getNEO(1));

    }
    resetEncoders();

    SmartDashboard.putNumber("DRPM2MPS", ModuleConstants.kDriveEncRPMperMPS);// .000645

    SmartDashboard.putNumber("DRFREE", ModuleConstants.kFreeMetersPerSecond);//

  }

  @Override
  public void periodic() {
    if (Pref.getPref("SwerveTune") == 1 && tuneOn == 0) {
      tuneGains();
      if (tuneOn == 1)
        tuneOn = (int) Pref.getPref("SwerveTune");
    }
    actualAngleDegrees = m_turningEncoder.getPosition();
    SmartDashboard.putNumber("APCF" + String.valueOf(m_moduleNumber), m_turningEncoder.getPositionConversionFactor());
  }

  public void tuneGains() {
    m_turnVelCotroller.setP(Pref.getPref("SwerveTurnVelkP"));
    m_turnVelCotroller.setI(Pref.getPref("SwerveTurnVelkI"));
    m_turnVelCotroller.setD(Pref.getPref("SwerveTurnVelKD"));
    m_turnVelCotroller.setIZone(Pref.getPref("SwerveTurnVelKIz"));
    tuneOn = 1;

  }

  @Override
  public void simulationPeriodic() {

    REVPhysicsSim.getInstance().run();

  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getHeadingDegrees()));
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

    state = SwerveModuleState.optimize(desiredState, new Rotation2d(actualAngleDegrees));

    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
        ? m_lastAngle
        : state.angle.getDegrees();

    m_lastAngle = angle;

    // actualAngleDegrees = wrapAngleDeg(m_turningEncoder.getPosition());

    if (angleChange) {

      actualAngleDegrees = m_turningEncoder.getPosition();

    }
    if (m_moduleNumber == 1)

      SmartDashboard.putNumber("OtPANG", optimumAngleDegrees - actualAngleDegrees);

    if (RobotBase.isReal()) {

      m_driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kFreeMetersPerSecond);

      double pidOut = m_turnPIDController.calculate(actualAngleDegrees, angle);

      m_turnVelCotroller.setReference(pidOut, ControlType.kVelocity);

      // m_turnVelCotroller.setReference(optimumAngleDegrees, ControlType.kPosition,
      // POS_SLOT);

    }

    if (RobotBase.isSimulation()) {

      m_driveVelController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, SIM_SLOT);

      double incrementper20ms;
      // angle change simulation
      if (angle != actualAngleDegrees) {

        incrementper20ms = initSimTurn(angle);

        if (angleSimInitted) {

          for (int i = 0; i <= simRamp; i++) {

            simAngle += incrementper20ms;

            simAngleCounter++;

            m_turningEncoder.setPosition(simAngle);
          }
          if (simAngleCounter > simRamp - 2)
            m_turningEncoder.setPosition(angle);
        }
      }
    }

  }

  public double initSimTurn(double angle) {

    double temp = shortRouteAngle(angle, m_turningEncoder.getPosition()) / simRamp;
    angleSimInitted = true;
    simAngle = 0;
    simAngleCounter = 0;
    return temp;

  }

  public double shortRouteAngle(double measurement, double setpoint) {
    double temp = setpoint;
    simAngleCounter = 0;
    if (Math.abs(setpoint - measurement) > 180)
      temp = MathUtil.inputModulus(setpoint - measurement, -180, 180);
    return temp;
  }

  public double wrapAngleDeg(double angle) {
    Math.IEEEremainder(angle, 360);
    angle = angle > 180 ? angle - 360 : angle;
    angle = angle < -180 ? angle + 360 : angle;
    return angle;
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

    tuLayout.addNumber("OptAngleEncDeg" + String.valueOf(m_moduleNumber), () -> optimumAngleDegrees);

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    // m_canEncoder.

  }

  public double getHeadingDegrees() {

    return m_turningEncoder.getPosition();

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

  // public double angleToEncoderRevs(double angle) {
  // return angle / ModuleConstants.kTurningDegreesPerEncRev;
  // }

  // public double encoderRevsToAngle(double encoderRevs) {
  // return encoderRevs * ModuleConstants.kTurningDegreesPerEncRev;
  // }

  // public double driveMPSToEncoderRPM(double driveMPS) {
  // return driveMPS * ModuleConstants.kDriveEncRPMperMPS;
  // }

  // public double encoderRPMToDriveMPS(double encoderRPM) {
  // return encoderRPM / ModuleConstants.kDriveEncRPMperMPS;
  // }

}
