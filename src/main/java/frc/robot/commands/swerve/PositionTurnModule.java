// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.DriveSubsystem;

public class PositionTurnModule extends CommandBase {

  private DriveSubsystem m_drive;


  
  private ModulePosition m_mp;

  /** Creates a new PositionTurnModule. */
  public PositionTurnModule(DriveSubsystem drive,  ModulePosition mp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;

    m_mp = mp;
  
    addRequirements(m_drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    m_drive.targetAngle = m_drive.getAnglefromThrottle();
    SmartDashboard.putNumber("TargetAngle", m_drive.targetAngle);
    m_drive.positionTurnModule(m_mp, m_drive.targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.turnModule(ModulePosition.FRONT_LEFT, 0);
    m_drive.turnModule(ModulePosition.FRONT_RIGHT, 0);
    m_drive.turnModule(ModulePosition.BACK_RIGHT, 0);
    m_drive.turnModule(ModulePosition.BACK_LEFT, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getTurnInPosition(m_mp, m_drive.targetAngle);
  }
}
