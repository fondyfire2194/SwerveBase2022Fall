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

                camLayout.addNumber("TargetsAvailable", () -> cam.targetsAvailable);

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

}