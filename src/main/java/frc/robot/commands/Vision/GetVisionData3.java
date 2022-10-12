// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cameras;

public class GetVisionData3 extends CommandBase {
  /** Creates a new GetVisionData. */
  private Cameras m_cam;

  public GetVisionData3(Cameras cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cam = cam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_cam.ptt2 = m_cam.trackedTargets.get(2);

    m_cam.grabTargetData(m_cam.ptt2, 2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
