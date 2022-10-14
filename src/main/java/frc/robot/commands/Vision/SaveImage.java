// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class SaveImage extends InstantCommand {
  PhotonCamera m_cam;
  boolean m_in;

  public SaveImage(PhotonCamera cam, boolean in) {
    m_in = in;m_cam=cam;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_in)
      m_cam.takeInputSnapshot();
    else
      m_cam.takeOutputSnapshot();
  }
}
