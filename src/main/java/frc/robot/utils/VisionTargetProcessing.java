// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Cameras;

/** Add your docs here. */
public class VisionTargetProcessing {
    public volatile boolean isStopped = false;
    int j;
    public PhotonPipelineResult plr;

    public VisionTargetProcessing(Cameras cam) {

        Thread processingThread = new Thread(new Runnable() {
            int t;

            @Override
            public void run() {

                while (!isStopped) {

                    // from an AprilTagID get the name and location of the target being processed

                    SmartDashboard.putNumber("THRNP", t++);

                }
            }

        });

        // Set up thread properties and start it off
        processingThread.setName("PhotonTargetProcessTask");
        processingThread.setPriority(Thread.MIN_PRIORITY);
        processingThread.start();
    }

}