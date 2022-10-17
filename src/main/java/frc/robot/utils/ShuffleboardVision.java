// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Cameras;
import frc.robot.commands.Vision.SetDriverMode;
import frc.robot.commands.Vision.SetPhotonPipeline;

/** Add your docs here. */
public class ShuffleboardVision {

        public ShuffleboardVision() {

        }

        public static void init(Cameras cam) {

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Cameras")
                                .getLayout("CameraLayout", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                camLayout.addNumber("ActivePipeline", () -> cam.llcam.getPipelineIndex());

                camLayout.addNumber("LatencySecs", () -> cam.latencySeconds);

                camLayout.addBoolean("DriverMode", () -> cam.llcam.getDriverMode());

                camLayout.addBoolean("HasTargets", () -> cam.hasTargets);

                camLayout.addNumber("TargetsAvailable", () -> cam.targetsAvailable);

                camLayout.addBoolean("UseBestOnly", ()->cam.bestTargetOnly);

                camLayout.addBoolean("Use3D", () -> cam.use3D);

                ShuffleboardLayout t0L;
                ShuffleboardLayout t1L;
                ShuffleboardLayout t2L;
                ShuffleboardLayout t0L3d;
                ShuffleboardLayout t1L3d;
                ShuffleboardLayout t2L3d;

                t0L = Shuffleboard.getTab("Cameras")
                                .getLayout("BestTarget", BuiltInLayouts.kList).withPosition(4, 0)

                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                if (!cam.use3D)

                        writeValues(t0L, cam, 0);

                t1L = Shuffleboard.getTab("Cameras")
                                .getLayout("Second Target", BuiltInLayouts.kList).withPosition(6, 0)

                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                if (!cam.use3D)

                        writeValues(t1L, cam, 1);

                t2L = Shuffleboard.getTab("Cameras")
                                .getLayout("ThirdTarget", BuiltInLayouts.kList).withPosition(8, 0)

                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                if (!cam.use3D)

                        writeValues(t2L, cam, 2);

                t0L3d = Shuffleboard.getTab("Cameras")
                                .getLayout("BestTarget3D", BuiltInLayouts.kList).withPosition(4, 3)

                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                if (cam.use3D)
                        write3DValues(t0L3d, cam, 0);

                t1L3d = Shuffleboard.getTab("Cameras")
                                .getLayout("Second Target3D", BuiltInLayouts.kList).withPosition(6, 3)

                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));
                                
                if (cam.use3D)
                        write3DValues(t1L3d, cam, 1);

                t2L3d = Shuffleboard.getTab("Cameras")
                                .getLayout("ThirdTarget3D", BuiltInLayouts.kList).withPosition(8, 3)

                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                if (cam.use3D)
                        write3DValues(t2L3d, cam, 2);

                ShuffleboardTab setPipeleine1 = Shuffleboard.getTab("Cameras");

                setPipeleine1.add("3DPipe", new SetPhotonPipeline(cam.llcam, 1))
                                .withPosition(2, 0).withSize(1, 1).withWidget(BuiltInWidgets.kCommand);
                setPipeleine1.add("2DPipe", new SetPhotonPipeline(cam.llcam, 0))
                                .withPosition(2, 1).withSize(1, 1).withWidget(BuiltInWidgets.kCommand);
                setPipeleine1.add("TurnOnDriverMode", new SetDriverMode(cam.llcam, true))
                                .withPosition(3, 0).withSize(1, 1).withWidget(BuiltInWidgets.kCommand);
                setPipeleine1.add("ResetDriverMode", new SetDriverMode(cam.llcam, false))
                                .withPosition(3, 1).withSize(1, 1).withWidget(BuiltInWidgets.kCommand);
                setPipeleine1.addBoolean("DriverMode", () -> cam.llcam.getDriverMode())
                                .withPosition(3, 2).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);
                setPipeleine1.addNumber("Pipeline", () -> cam.llcam.getPipelineIndex())
                                .withPosition(2, 2).withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

                // if (RobotBase.isReal())

                // {

                // ShuffleboardTab llvFeed = Shuffleboard.getTab("Cameras");

                // llvFeed.addCamera("LL", "camera", "http://10.21.94.11:5800/stream.mjpg")
                // .withPosition(0, 2).withSize(2, 4)
                // .withProperties(Map.of("Show Crosshair", true,
                // "Show Controls", true, "Rotation", "QUARTER_CW"));
                // }

        }

        public static void writeValues(ShuffleboardLayout tnL, Cameras cam, int n) {

                int temp = cam.tagID[n];

                tnL.addString("Location", () -> cam.targetLocationNames[temp]);

                tnL.addNumber("AprilTagID", () -> cam.tagID[n]);

                tnL.addNumber("TargetYaw", () -> cam.yaw[n]);

                tnL.addNumber("TargetPitch", () -> cam.pitch[n]);

                tnL.addNumber("TargetSkew", () -> cam.skew[n]);

                tnL.addNumber("TargetArea", () -> cam.area[n]);

                tnL.addNumber("PoseAmbiguity", () -> cam.poseAmbiguity[n]);

        }

        public static void write3DValues(ShuffleboardLayout tnL, Cameras cam, int n) {

                tnL.addNumber("3D-ToCam X", () -> cam.X[n]);

                tnL.addNumber("3D-ToCam Y", () -> cam.Y[n]);

                tnL.addNumber("3D-ToCam Z", () -> cam.Z[n]);

        }

}