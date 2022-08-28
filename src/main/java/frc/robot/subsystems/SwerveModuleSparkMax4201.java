// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;

public class SwerveModuleSparkMax4201 {
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  public double[] data = new double[4];

  private final CTRECanCoder m_canEncoder;
  public double commandAngle;
  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  ModulePosition m_modulePosition;
  int m_moduleNumber;
  // Using a TrapezoidProfile PIDController to allow for smooth turning
  public ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          ModuleConstants.MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED));

  private MechanismLigament2d m_cmdvec;
  private MechanismLigament2d m_actvec;
  private MechanismLigament2d m_rotcmdvec;
  private MechanismLigament2d m_rotactvec;

  // the main mechanism object
  public Mechanism2d mech = new Mechanism2d(4, 4);
  // the mechanism root node
  MechanismRoot2d root = mech.getRoot("fl", 2, 2);
  Pose2d m_pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  public static final DCMotor kDriveGearbox = DCMotor.getNEO(1);
  public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);
  public static final double kDriveMotorGearRatio = 6.12;
  public static final double kTurningMotorGearRatio = 12.8;
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
  private final FlywheelSim m_turnMotorSim = new FlywheelSim(
      // Sim Values
      LinearSystemId.identifyVelocitySystem(0.1, 0.0008), kTurnGearbox, kTurningMotorGearRatio);

  private final FlywheelSim m_driveMotorSim = new FlywheelSim(
      // Sim Values
      LinearSystemId.identifyVelocitySystem(4, 1.24), kDriveGearbox, kDriveMotorGearRatio);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param driveEncoderChannels   The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModuleSparkMax4201(
      ModulePosition modulePosition,
      int driveMotorCanChannel,
      int turningMotorCanChannel,
      int cancoderCanChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed) {

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
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningRadiansPerPulse);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_cmdvec = root.append(new MechanismLigament2d("cmd", 0, 0, 6, new Color8Bit(Color.kPurple)));
    m_actvec = root.append(new MechanismLigament2d("act", 0, 0, 6, new Color8Bit(Color.kYellow)));
    m_rotcmdvec = root.append(new MechanismLigament2d("rotcmd", 0, 0, 6, new Color8Bit(Color.kGreen)));
    m_rotactvec = root.append(new MechanismLigament2d("rotact", 0, 0, 6, new Color8Bit(Color.kBrown)));

    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  public ModulePosition getModulePosition() {
    return m_modulePosition;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    commandAngle = desiredState.angle.getDegrees();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
        state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
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

  public void showMec() {
    SmartDashboard.putData(getModulePosition().toString(), mech);
  }

  public void showData() {
    
    SmartDashboard.putNumberArray(getModulePosition().toString()+" Data", data);
  }

  public void updateGlass() {

    data[0] = m_driveMotor.get();
    data[1] = commandAngle;
    data[2] = getState().speedMetersPerSecond;
    data[3] = getState().angle.getDegrees();

    m_cmdvec.setLength(m_driveMotor.get());
    m_cmdvec.setAngle(commandAngle);

    m_actvec.setLength(getState().speedMetersPerSecond);
    m_actvec.setAngle(getState().angle.getDegrees());

    m_rotactvec.setLength(m_turningEncoder.getPosition());
    m_rotcmdvec.setAngle(m_turningMotor.get());

  }

}
