// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.AprilTagData;
import frc.robot.utils.VisionTargetGrabber;

/** Add your docs here. */
public class Cameras {

    public PhotonCamera llcam = new PhotonCamera("camera");
    // public PhotonCamera llcam = new PhotonCamera("limelight");

    public boolean use3D = false;

    public double yaw[] = { 0, 0, 0 };
    public int tagID[] = { 0, 0, 0 };
    public double pitch[] = { 0, 0, 0 };
    public double area[] = { 0, 0, 0 };
    public double skew[] = { 0, 0, 0 };
    public double X[] = { 0, 0, 0 };
    public double Y[] = { 0, 0, 0 };
    public double Z[] = { 0, 0, 0 };
    public double A[] = { 0, 0, 0 };

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
    public PhotonTrackedTarget ptt2;

    public Transform3d[] tag = new Transform3d[3];

    public boolean bestTargetOnly = false;

    public String[] targetLocationNames = { "", "", "" };

    public Cameras() {

        llcam.setLED(VisionLEDMode.kOff);

        VisionTargetGrabber visGrab = new VisionTargetGrabber(this);

        // VisionTargetProcessing visPorc = new VisionTargetProcessing(this);

        tag[0] = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        tag[1] = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        tag[2] = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

        clear2dResults();

        clear3dResults();
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

    public void grabTargetData(PhotonTrackedTarget ptt, int i) {

        if (!use3D) {
            tagID[i] = ptt.getFiducialId();
            yaw[i] = ptt.getYaw();
            pitch[i] = ptt.getPitch();
            skew[i] = ptt.getSkew();
            area[i] = ptt.getArea();
            poseAmbiguity[i] = ptt.getPoseAmbiguity();

        } else {

            Transform3d ctoT = ptt.getCameraToTarget();
            X[i] = ctoT.getX();
            Y[i] = ctoT.getY();
            Z[i] = ctoT.getZ();

        }
        tag[i] = AprilTagData.getTransform3d(tagID[i]);
    }

    public void clear2dResults() {

        for (int i = 0; i < yaw.length - 1; i++) {
            yaw[i] = 0;
            pitch[i] = 0;
            skew[i] = 0;
            area[i] = 0;
            tagID[i] = 0;
            targetsAvailable = 0;
            targetLocationNames[i] = "Not a Location";

        }
    }

    public void clear3dResults() {

        for (int i = 0; i < yaw.length - 1; i++) {
            X[i] = 0;
            Y[i] = 0;
            Z[i] = 0;
            rotation[i] = "NoTarget";
            targetsAvailable = 0;
            targetLocationNames[i] = "Not a Location";
        }

    }

    public void periodic() {

        use3D = llcam.getPipelineIndex() == 1;

        if (use3D && area[0] != 0)
            clear2dResults();

        if (!use3D && X[0] != 0)
            clear3dResults();

        SmartDashboard.putNumber("PIDX", llcam.getPipelineIndex());

        SmartDashboard.putNumber("TSTN ", 911);
    }
}
