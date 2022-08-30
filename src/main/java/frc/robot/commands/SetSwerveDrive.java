package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class SetSwerveDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_swerveDrive;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  private final boolean m_fieldOriented, m_openLoop;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDrive(
      DriveSubsystem swerveDriveSubsystem,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput,
      boolean fieldOriented,
      boolean openLoop) {
    m_swerveDrive = swerveDriveSubsystem;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_fieldOriented = fieldOriented;
    m_openLoop = openLoop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
        * Math.signum(m_throttleInput.getAsDouble());
    double strafe = MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
        * Math.signum(m_strafeInput.getAsDouble());
    double rotation = MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
        * Math.signum(m_rotationInput.getAsDouble());
    SmartDashboard.putNumber("TH", throttle);
    SmartDashboard.putNumber("ST", strafe);
    SmartDashboard.putNumber("ROT", rotation);

    m_swerveDrive.drive(throttle, strafe, rotation, m_fieldOriented, m_openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
