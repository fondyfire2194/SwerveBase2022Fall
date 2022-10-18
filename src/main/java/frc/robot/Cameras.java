// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class Cameras {

    public PhotonCamera llcam = new PhotonCamera("camera");
    // public PhotonCamera llcam = new PhotonCamera("limelight");

 

    public double poseAmbiguity[] = { 0, 0, 0, 0 };
    public String rotation[] = { "0", "0", "0", "0" };
    public int targetsAvailable = 0;
    public boolean hasTargets;
    public double latencySeconds;
    int i = 0;
    int tst;
    public int targetToProcess;

    public boolean targetActive = true;
    public List<PhotonTrackedTarget> trackedTargets;
    public PhotonPipelineResult plr;
    public PhotonTrackedTarget ptt0;
    public PhotonTrackedTarget ptt1;
  

    public Transform3d[] tag = new Transform3d[3];

    public String[] targetLocationNames = { "", "", "" };

    public Cameras() {

        llcam.setLED(VisionLEDMode.kOff);

          }

    public PhotonPipelineResult getLatestResult() {
        return llcam.getLatestResult();
    }

    public boolean getHasTargets(PhotonPipelineResult plr) {
        return plr.hasTargets();
    }

    public List<PhotonTrackedTarget> getTrackedTargets(PhotonPipelineResult plr) {
        return plr.getTargets();
    }

    public int getNumberTargets(PhotonPipelineResult plr) {
        return plr.targets.size();
    }

    public double getLatencySeconds(PhotonPipelineResult plr) {
        return plr.getLatencyMillis();
    }

    public Transform3d getCameraToTarget() {
        return getLatestResult().getBestTarget().getCameraToTarget();
    }


    


    public void periodic() {

           }
}
