// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.subsystems.DriveSubsystem;

public class PositionTurnModule extends CommandBase {

  private DriveSubsystem m_drive;
  private ModulePosition m_mp;
  private double m_targetAngle;
  

  
  /** Creates a new PositionTurnModule. */
  public PositionTurnModule(DriveSubsystem drive, ModulePosition mp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive=drive;
    m_mp =mp;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetAngle = m_drive.throttleValue * 180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.positionTurnModule(m_mp, m_targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getTurnInPosition(m_mp, m_targetAngle);
  }
}
