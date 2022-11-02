// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDStrip;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightStrip;

public class ForceAllianceColor extends CommandBase {
  /** Creates a new ForceAllianceColor. */
  private LightStrip m_strip;
  private boolean m_blueAlliance;

  public ForceAllianceColor(LightStrip strip, boolean blueAlliance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_strip = strip;
    m_blueAlliance = blueAlliance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_strip.allianceColorSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_strip.forceAllianceColor(m_blueAlliance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_strip.allianceColorSet;
  }
}
