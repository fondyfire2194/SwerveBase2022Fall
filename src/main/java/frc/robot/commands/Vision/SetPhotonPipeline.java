// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPhotonPipeline extends CommandBase {
  private PhotonCamera m_cam;
  private int m_number;
  private int loopctr;

  public SetPhotonPipeline(PhotonCamera cam, int number) {
    // Use addRequirements() here t can,o declare subsystem dependencies.
    m_cam = cam;
    m_number = number;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_cam.getPipelineIndex() != m_number)
      m_cam.setPipelineIndex(m_number);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_cam.getPipelineIndex() == m_number;
  }

}
