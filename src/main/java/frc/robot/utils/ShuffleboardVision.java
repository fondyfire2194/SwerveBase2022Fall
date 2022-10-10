// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.PhotonVisionCam;

/** Add your docs here. */
public class ShuffleboardVision {
        public static int n = 1;

        public ShuffleboardVision() {

        }

        public static void init(PhotonVisionCam pvcam, int camNum) {

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Cameras")
                                .getLayout("CameraLayout", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                camLayout.addNumber("ActivePipeline", () -> pvcam.getActivePipeline());

                camLayout.addNumber("NumberTargets", () -> pvcam.getNumberTargets());

                camLayout.addNumber("LatencySecs", () -> pvcam.getLatencySec());

                camLayout.addBoolean("DriverMode", () -> pvcam.getDriverMode());

                camLayout.addBoolean("HasTargets", () -> pvcam.hasTargets());

                n = 1;

                ShuffleboardLayout t1L;

                t1L = Shuffleboard.getTab("Cameras")
                                .getLayout("Target" + String.valueOf(n), BuiltInLayouts.kList).withPosition(n * 2, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "TOP"));

                t1L.addNumber("AprilTagID", () -> pvcam.getAprilTagID(n));

                t1L.addNumber("TargetYaw", () -> pvcam.getTargetYaw(n));

                t1L.addNumber("TargetPitch", () -> pvcam.getTargetPitch(n));

                t1L.addNumber("TargetSkew", () -> pvcam.getTargetSkew(n));

                t1L.addNumber("TargetArea", () -> pvcam.getTargetArea(n));

                t1L.addNumber("ToCam X", () -> pvcam.getCameraToTarget(n).getX());

                t1L.addNumber("ToCam Y", () -> pvcam.getCameraToTarget(n).getY());

                t1L.addNumber("ToCam Z", () -> pvcam.getCameraToTarget(n).getZ());

                t1L.addString("ToCam Rd3", () -> pvcam.getCameraToTarget(n).getRotation().toString());


        }
}