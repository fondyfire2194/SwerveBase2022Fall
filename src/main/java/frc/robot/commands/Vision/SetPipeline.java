// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPipeline extends CommandBase {
  private PhotonCamera m_cam;
  private int m_num;;

  public SetPipeline(PhotonCamera cam, int num) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cam = cam;
    m_num = num;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cam.setPipelineIndex(m_num);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_cam.getPipelineIndex() == m_num;
  }
}
