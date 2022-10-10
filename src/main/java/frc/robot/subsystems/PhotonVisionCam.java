// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardVision;

public class PhotonVisionCam extends SubsystemBase {
  String m_cameraName;
  PhotonCamera m_camera;
  int m_camNum;

  /** Creates a new PhotonVision. */
  public PhotonVisionCam(String cameraName,int  camNum) {

    m_cameraName = cameraName;
    m_camNum = camNum;
    m_camera = new PhotonCamera(cameraName);
    ShuffleboardVision.init(this,m_camNum);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  public List<PhotonTrackedTarget> getTrackedTargets() {
    return m_camera.getLatestResult().getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    return m_camera.getLatestResult().getBestTarget();
  }

  public int getNumberTargets() {
    return getTrackedTargets().size();
  }

  public int getAprilTagID(int target) {
    return getTrackedTargets().get(target).getFiducialId();
  }

  public Transform3d getATx(int target) {
    return getTrackedTargets().get(target).getCameraToTarget();
  }

  public boolean hasTargets() {
    return m_camera.getLatestResult().hasTargets();
  }

  public void setDriverMode(boolean on) {
    m_camera.setDriverMode(on);
  }

  public boolean getDriverMode() {
    return m_camera.getDriverMode();
  }

  public int getActivePipeline() {
    return m_camera.getPipelineIndex();
  }

  public void setActivePipeline(int n) {
    m_camera.setPipelineIndex(n);
  }

  public void takeInputSnapshot() {
    m_camera.takeInputSnapshot();
  }

  public void takeOutputSnapshot() {
    m_camera.takeOutputSnapshot();
  }

  public double getLatencySec() {
    return m_camera.getLatestResult().getLatencyMillis()/1000;
  }


  public double getTargetYaw(int target) {
    return getTrackedTargets().get(target).getYaw();
  }

  public double getTargetArea(int target) {
    return getTrackedTargets().get(target).getArea();
  }

  public double getTargetPitch(int target) {
    return getTrackedTargets().get(target).getPitch();
  }

  public double getTargetSkew(int target) {
    return getTrackedTargets().get(target).getSkew();
  }

  public Transform3d getCameraToTarget(int target) {
    return getTrackedTargets().get(target).getCameraToTarget();
  }

  public List<TargetCorner> getTargetCorners(int target) {
    return getTrackedTargets().get(target).getCorners();
  }

}