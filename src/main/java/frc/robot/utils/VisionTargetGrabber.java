// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Cameras;

/** Add your docs here. */
public class VisionTargetGrabber {
    public volatile boolean isStopped = false;
    int j;
    public PhotonPipelineResult plr;

    public VisionTargetGrabber(Cameras cam) {

        Thread grabberThread = new Thread(new Runnable() {
            int t;

            @Override
            public void run() {

                while (!isStopped) {

                    SmartDashboard.putNumber("THRNG", t++);

                  //  plr = cam.getLatestResult();

                    // if (plr.hasTargets()) {
                    //     t++;

                        // j = cam.getNumberTargets();

                        // cam.grabTargetData(plr,0);

                        // if (j >= 1)
                        // cam.grabTargetData(plr,1);

                        // if (j >= 2)
                        // cam.grabTargetData(plr,2);

                  //  }

                    try {

                        Thread.sleep(10);

                    } catch (InterruptedException e) {
                        SmartDashboard.putBoolean("OOPS", true);
                    }

                }
            }
        });

        // Set up thread properties and start it off
        grabberThread.setName("PhotonTargetGrabTask");
        grabberThread.setPriority(Thread.MAX_PRIORITY);
        grabberThread.start();
    }

}