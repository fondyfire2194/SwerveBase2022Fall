package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SetSwerveDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_swerveDrive;
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);
  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDrive(
      DriveSubsystem swerveDriveSubsystem,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {
    m_swerveDrive = swerveDriveSubsystem;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // https://www.chiefdelphi.com/t/swerve-controller-joystick/392544/5
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()),
        DriveConstants.kControllerDeadband)
        * Math.signum(m_throttleInput.getAsDouble());
    double strafe = MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()),
        DriveConstants.kControllerDeadband)
        * Math.signum(m_strafeInput.getAsDouble());
    double rotation = MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()),
        DriveConstants.kControllerRotDeadband)
        * Math.signum(m_rotationInput.getAsDouble());

    // square values after deadband while keeping original sign

    throttle = -Math.signum(throttle) * Math.pow(throttle, 2);
    strafe = -Math.signum(strafe) * Math.pow(strafe, 2);
    rotation = -Math.signum(rotation) * Math.pow(rotation, 2);

    double throttle_sl = m_slewX.calculate(throttle);
    double strafe_sl = m_slewY.calculate(strafe);
    double rotation_sl = m_slewRot.calculate(rotation);

    m_swerveDrive.drive(throttle_sl, strafe_sl, rotation_sl, true);

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
