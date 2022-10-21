// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cameras;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetDriverMode extends CommandBase {
  private PhotonCamera m_cam;
  private boolean m_on;
  private double startTime;

  public SetDriverMode(PhotonCamera cam, boolean on) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cam = cam;
    m_on = on;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_cam.getDriverMode() == m_on)
      m_cam.setDriverMode(m_on);

    if (m_cam.getDriverMode() == m_on && startTime == 0) {
      startTime = Timer.getFPGATimestamp();
    } else
      startTime = 0;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return startTime != 0 && Timer.getFPGATimestamp() > startTime + 1 &&m_cam.getDriverMode()==m_on;
  }
}