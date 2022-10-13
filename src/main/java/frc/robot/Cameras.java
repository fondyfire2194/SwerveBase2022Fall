// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.AprilTagData;
import frc.robot.utils.ShuffleboardVision;
import frc.robot.utils.VisionTargetGrabber;

/** Add your docs here. */
public class Cameras {

   // public PhotonCamera picam = new PhotonCamera("picamrpi4");
    public PhotonCamera picam = new PhotonCamera("limelight");
    public double yaw[] = { 0, 0, 0 };
    public int tagID[] = { 0, 0, 0 };
    public double pitch[] = { 0, 0, 0 };
    public double area[] = { 0, 0, 0 };
    public double skew[] = { 0, 0, 0 };
    public double X[] = { 0, 0, 0 };
    public double Y[] = { 0, 0, 0 };
    public double Z[] = { 0, 0, 0 };
    public double poseAmbiguity[] = { 0, 0, 0, 0 };
    public String rotation[] = { "0", "0", "0" };
    public int targetsAvailable = 0;
    public boolean hasTargets;
    public double latencySeconds;
    public static int idx;
    int i = 0;
    int tst;

    public boolean targetActive = true;
    public List<PhotonTrackedTarget> trackedTargets;
    public PhotonPipelineResult plr;
    public PhotonTrackedTarget ptt0;
    public PhotonTrackedTarget ptt1;
    public PhotonTrackedTarget ptt2;

    public Translation3d tag1;
    public Translation3d tag2;
    public Translation3d tag3;  

    public Cameras() {
        ShuffleboardVision.init(this);
        
        AprilTagData.init();
        VisionTargetGrabber vis=new VisionTargetGrabber(this);
        
    }

    public PhotonPipelineResult getLatestResult() {
        return picam.getLatestResult();
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

    public PhotonTrackedTarget getTrackedTarget(List<PhotonTrackedTarget> lptt, int targetNumber) {
        return lptt.get(targetNumber);
    }

    public double getLatencySeconds(PhotonPipelineResult plr) {
        return plr.getLatencyMillis();
    }

    public void getBestTargetData(PhotonPipelineResult plr) {

     PhotonTrackedTarget btt=   plr.getBestTarget();
        SmartDashboard.putString("PTT" + String.valueOf(i), btt.toString());
        tagID[i] = btt.getFiducialId();
        yaw[i] = btt.getYaw();
        pitch[i] = btt.getPitch();
        skew[i] = btt.getSkew();
        area[i] = btt.getArea();
        poseAmbiguity[i] = btt.getPoseAmbiguity();
    }

    public void grabTargetData(PhotonTrackedTarget ptt, int i) {
        SmartDashboard.putString("PTT" + String.valueOf(i), ptt.toString());
        tagID[i] = ptt.getFiducialId();
        yaw[i] = ptt.getYaw();
        pitch[i] = ptt.getPitch();
        skew[i] = ptt.getSkew();
        area[i] = ptt.getArea();
        poseAmbiguity[i] = ptt.getPoseAmbiguity();

        // Transform3d ctoT = ptt.getCameraToTarget();
        // X[i] = ctoT.getX();
        // Y[i] = ctoT.getY();
        // Z[i] = ctoT.getZ();

        // rotation[i] = ctoT.getRotation().toString();
    }

    public void clearResults() {

        for (int i = 0; i < yaw.length - 1; i++) {
            yaw[i] = 0;
            pitch[i] = 0;
            skew[i] = 0;
            area[i] = 0;
            tagID[i] = 0;
            X[i] = 0;
            Y[i] = 0;
            Z[i] = 0;
            rotation[i] = "NoTarget";
            targetsAvailable = 0;
        }


    }

    public static void periodic(){
        

    }
    // double temp = 0;
    // temp = viewTarget.getDouble(0);
    // idx = (int) temp;
}