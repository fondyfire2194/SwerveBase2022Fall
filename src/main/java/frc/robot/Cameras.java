// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Size;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardVision;

/** Add your docs here. */
public class Cameras extends SubsystemBase {

    public PhotonCamera picam = new PhotonCamera("picamrpi4");
    public double yaw;
    public int tagID;
    public double pitch;
    public double area;
    public double skew;
    public double X;
    public double Y;
    public double Z;
    public String rotation;
    public int targetsAvailable;

    public Cameras() {
        ShuffleboardVision.init(this);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        if (picam.getLatestResult().hasTargets()) {
            targetsAvailable=picam.getLatestResult().targets.size();
            PhotonTrackedTarget ptt = picam.getLatestResult().getBestTarget();

            yaw = ptt.getYaw();
            pitch = ptt.getPitch();
            skew = ptt.getSkew();
            area = ptt.getArea();
            Transform3d ctoT = ptt.getCameraToTarget();
            X = ctoT.getX();
            Y = ctoT.getY();
            Z = ctoT.getZ();

            rotation = ctoT.getRotation().toString();

        } else {
            yaw = 0;
            pitch = 0;
            skew = 0;
            area = 0;
            tagID = 0;
            X = 0;
            Y = 0;
            Z = 0;
            rotation = "NoTarget";
            targetsAvailable=0;
        }

    }

    @Override
    public void simulationPeriodic() {
    }
}
