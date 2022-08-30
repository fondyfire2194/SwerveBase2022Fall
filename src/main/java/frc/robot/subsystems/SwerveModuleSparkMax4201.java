// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Sim.CANEncoderSim;
import frc.robot.Sim.CANSparkMaxWithSim;

public class SwerveModuleSparkMax4201 extends SubsystemBase {
  public final CANSparkMaxWithSim m_driveMotor;
  public final CANSparkMaxWithSim m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private CANEncoderSim m_driveSimEncoder = null;;
  private CANEncoderSim m_turnSimEncoder = null;

  private final CTRECanCoder m_canEncoder;
  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  ModulePosition m_modulePosition;
  int m_moduleNumber;
  public String layout;
  // Using a TrapezoidProfile PIDController to allow for smooth turning
  public ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          ModuleConstants.MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED));

  Pose2d m_pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);

  final LinearSystem<N1, N1, N1> m_drive = LinearSystemId
      .identifyVelocitySystem(.0774, 0.0005);
  LinearSystemSim<N1, N1, N1> m_driveSim = new LinearSystemSim<>(m_drive);

  final LinearSystem<N1, N1, N1> m_turn = LinearSystemId
      .identifyVelocitySystem(.0774, 0.0005);
  LinearSystemSim<N1, N1, N1> m_turnSim = new LinearSystemSim<>(m_turn);

  private double commandTurnAngle;
  private double actTurnAngle;
  private double commandDriveSpeed;
  private double actDriveSpeed;
  double tpos;
  public ShuffleboardLayout varLayout;
  private double m_turnSimDistance;
  private double m_turnSimTarget;

  // private double m_driveMotorSimDistance;
  // private double m_turnMotorSimDistance;
  // private double m_drivePercentOutput;
  // private double m_turnPercentOutput;

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

    m_driveMotor = new CANSparkMaxWithSim(driveMotorCanChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMaxWithSim(turningMotorCanChannel, MotorType.kBrushless);

    m_driveMotor.setInverted(driveMotorReversed);
    m_turningMotor.setInverted(turningMotorReversed);

    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_driveMotor.setIdleMode(CANSparkMaxWithSim.IdleMode.kBrake);

    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_turningMotor.setIdleMode(CANSparkMaxWithSim.IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_turningEncoder = m_turningMotor.getEncoder();

    m_canEncoder = new CTRECanCoder(cancoderCanChannel);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerPulse);

    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMetersPerPulse / 60);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerPulse);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-180, 180);

    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();

    if (RobotBase.isSimulation()) {
      m_driveSimEncoder = new CANEncoderSim(m_driveMotor.getDeviceId(), driveMotorReversed);
      m_turnSimEncoder = new CANEncoderSim(m_turningMotor.getDeviceId(), turningMotorReversed);
    }
    resetEncoders();
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("TVel" + String.valueOf(m_moduleNumber), m_turningMotor.get());

    m_turnSim.setInput(m_turningMotor.get() * 12);

    m_driveSim.setInput(m_driveMotor.get() * 12);

    m_turnSim.update(0.02);
    m_driveSim.update(0.02);

    Unmanaged.feedEnable(20);

    if (Math.abs(m_turnSimTarget) < 360)
      m_turnSimTarget += m_driveSim.getOutput(0) * .002;

    m_turnSimDistance += m_turningMotor.get() * .2;

    wrapAngleDeg(m_turnSimDistance);
    wrapAngleDeg(m_turnSimTarget);
    SmartDashboard.putNumber("Vel" + String.valueOf(m_moduleNumber), m_driveSim.getOutput(0));

    SmartDashboard.putNumber("TuTGT" + String.valueOf(m_moduleNumber), m_turnSimTarget);
    SmartDashboard.putNumber("TuPos" + String.valueOf(m_moduleNumber), m_turnSimDistance);
    SmartDashboard.putString("TuTGTR2d" + String.valueOf(m_moduleNumber), getHeadingRotation2d().toString());
    m_driveSimEncoder.setPosition(m_turnSimDistance);
    actDriveSpeed = m_driveSim.getOutput(0);

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    actDriveSpeed = m_driveEncoder.getVelocity();
    if (RobotBase.isSimulation())
      actDriveSpeed = m_driveSim.getOutput(0);

    actTurnAngle = m_turningEncoder.getPosition();
    if (RobotBase.isSimulation())
      actTurnAngle = m_turnSimDistance;
    wrapAngleDeg(actTurnAngle);
    return new SwerveModuleState(actDriveSpeed, new Rotation2d(actTurnAngle));
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

    actTurnAngle = m_turningEncoder.getPosition();

    if (RobotBase.isSimulation()) {
      commandTurnAngle = wrapAngleDeg(m_turnSimTarget);
      actTurnAngle = wrapAngleDeg(m_turnSimDistance);

    }

    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(actTurnAngle));

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turnSimDistance,
        m_turnSimTarget);

    m_turningMotor.set(turnOutput);

    // Calculate the drive output from the drive PID controller.
    commandDriveSpeed = state.speedMetersPerSecond;
    actDriveSpeed = m_driveEncoder.getVelocity();
    commandTurnAngle = state.angle.getDegrees();
    if (RobotBase.isSimulation())
      actDriveSpeed = m_driveSim.getOutput(0);
    final double driveOutput = m_drivePIDController.calculate(actDriveSpeed, state.speedMetersPerSecond);

    if (isOpenLoop) {

      m_driveMotor.set(state.speedMetersPerSecond / 18);// ModuleConstants.kMaxSpeedMetersPerSecond;
    } else
      m_driveMotor.set(driveOutput);
  }

  public void initShuffleboard() {
    layout = m_modulePosition.toString();
    ShuffleboardLayout xLayout = Shuffleboard.getTab("Drivetrain")
        .getLayout(layout, BuiltInLayouts.kList).withPosition(m_moduleNumber * 2, 0)
        .withSize(2, 5).withProperties(Map.of("Label position", "TOP"));

    xLayout.addNumber("Turn Setpoint Degrees " + String.valueOf(m_moduleNumber), () -> getState().angle.getDegrees());
    xLayout.addNumber("Turn Angle Degrees " + String.valueOf(m_moduleNumber), () -> actTurnAngle);
    xLayout.addNumber("Drive Setpoint MPS " + String.valueOf(m_moduleNumber), () -> getState().speedMetersPerSecond);
    xLayout.addNumber("Drive Speed MPS " + String.valueOf(m_moduleNumber), () -> actDriveSpeed);

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    m_turnSimDistance = 0;
    m_turnSimTarget = 0;
    actTurnAngle=0;
    commandTurnAngle=0;
  }

  public double getHeadingDegrees() {
    if (RobotBase.isReal())
      return m_turningEncoder.getPosition();
    else
      return m_turnSimDistance;
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
