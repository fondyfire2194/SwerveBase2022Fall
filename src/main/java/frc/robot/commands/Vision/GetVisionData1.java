// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.opencv.core.Size;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cameras;

public class GetVisionData1 extends CommandBase {
  /** Creates a new GetVisionData. */
  private Cameras m_cam;

  boolean hasTargets;
  int numberTargets;
  int loopCtr;

  public GetVisionData1(Cameras cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cam = cam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_cam.plr = m_cam.getLatestResult();


    m_cam.hasTargets = m_cam.getHasTargets(m_cam.plr);

    if (m_cam.hasTargets) {

      m_cam.trackedTargets = m_cam.getTrackedTargets(m_cam.plr);

      m_cam.targetsAvailable = m_cam.trackedTargets.size();

      m_cam.latencySeconds = m_cam.getLatencySeconds(m_cam.plr);

      m_cam.ptt0 = m_cam.getTrackedTarget(m_cam.trackedTargets, 0);

      

      m_cam.grabTargetData(m_cam.ptt0, 0);
    }

    else
      m_cam.clearResults();

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
