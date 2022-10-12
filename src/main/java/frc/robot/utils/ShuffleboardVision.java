// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Cameras;

/** Add your docs here. */
public class ShuffleboardVision {
        public static int n = 1;

        public ShuffleboardVision() {

        }

        public static void init(Cameras cam) {

                PhotonCamera picam = cam.picam;

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Cameras")
                                .getLayout("CameraLayout", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                camLayout.addNumber("ActivePipeline", () -> picam.getPipelineIndex());

                camLayout.addNumber("LatencySecs", () -> picam.getLatestResult().getLatencyMillis() / 1000);

                camLayout.addBoolean("DriverMode", () -> picam.getDriverMode());

                camLayout.addBoolean("HasTargets", () -> picam.getLatestResult().hasTargets());

                camLayout.addNumber("TargetsAvailable", ()->cam.targetsAvailable);

                ShuffleboardLayout t1L;

                t1L = Shuffleboard.getTab("Cameras")
                                .getLayout("Best Target", BuiltInLayouts.kList).withPosition(2,0)
                                                 
                                .withSize(6, 4).withProperties(Map.of("Label position", "LEFT"));

                

                t1L.addNumber("AprilTagID", () -> cam.tagID);

                t1L.addNumber("TargetYaw", () -> cam.yaw);

                t1L.addNumber("TargetPitch", () -> cam.pitch);

                t1L.addNumber("TargetSkew", () -> cam.skew);

                t1L.addNumber("TargetArea", () -> cam.area);

                 t1L.addNumber("3D-ToCam X", () -> cam.X);

                 t1L.addNumber("3D-ToCam Y", () -> cam.Y);

                 t1L.addNumber("3D-ToCam Z", () -> cam.Z);

                 t1L.addString("3D-ToCam Rd3", () -> cam.rotation);

        }
}