// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Vision.SetEstPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimatorSubsystem;

/** Add your docs here. */
public class ShuffleboardVisionTest {

        

        public static void init(VisionPoseEstimatorSubsystem vpe, DriveSubsystem drive) {
                // NetworkTableInstance inst = NetworkTableInstance.getDefault();
                // NetworkTable table = inst.getTable("PoseEst");
                // NetworkTableEntry xEntry = table.getEntry("x");
                // NetworkTableEntry yEntry = table.getEntry("y");

                ShuffleboardLayout camLayout = Shuffleboard.getTab("CamerasTest")
                                .getLayout("CameraLayout", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(7, 1).withProperties(Map.of("Label position", "LEFT"));

                camLayout.addString("Target 1", () -> VisionPoseEstimatorSubsystem.targetPoses.get(4).toString());
              //  camLayout.addString("Target 2", () -> VisionPoseEstimatorSubsystem.targetPoses.get(1).toString());
                camLayout.addString("Cam2Rob", () -> VisionConstants.CAMERA_TO_ROBOT_3D.toString());

                // camLayout.addString("Cam2Target", () ->
                // VisionPoseEstimatorSubsystem.camToTarget.toString());
                camLayout.addString("Cam2TargetCorr", () -> VisionPoseEstimatorSubsystem.camToTarget.toString());
                camLayout.addString("CamPose", () -> VisionPoseEstimatorSubsystem.camPose.toString());
                camLayout.addString("VisCorr", () -> VisionPoseEstimatorSubsystem.visionMeasurement.toString());

                ShuffleboardLayout datLayout = Shuffleboard.getTab("CamerasTest")
                                .getLayout("DataLayout", BuiltInLayouts.kList)
                                .withPosition(7, 0)
                                .withSize(7, 4).withProperties(Map.of("Label position", "LEFT"));

                datLayout.add("SetEstPosn", new SetEstPosition(drive, 1, 2, 3));

        }

}