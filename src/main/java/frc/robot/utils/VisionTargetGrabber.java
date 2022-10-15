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

                    cam.plr = cam.getLatestResult();

                    cam.hasTargets = cam.getHasTargets(cam.plr);

                    SmartDashboard.putBoolean("HasTgts", cam.hasTargets);

                    if (cam.hasTargets) {

                        if (!cam.bestTargetOnly) {

                            cam.trackedTargets = cam.getTrackedTargets(cam.plr);

                            cam.targetsAvailable = cam.trackedTargets.size();

                            cam.latencySeconds = cam.getLatencySeconds(cam.plr);

                            cam.ptt0 = cam.getTrackedTarget(cam.trackedTargets, 0);

                            cam.grabTargetData(cam.ptt0, 0);

                            int temp = cam.tagID[0];

                            cam.tag1 = AprilTagData.getTransform3d(temp);

                            if (cam.targetsAvailable >= 2) {

                                cam.ptt1 = cam.trackedTargets.get(1);

                                cam.grabTargetData(cam.ptt1, 1);

                                temp = cam.tagID[1];

                                cam.tag2 = AprilTagData.getTransform3d(temp);
                            }

                            if (cam.targetsAvailable > 2) {

                                cam.ptt2 = cam.trackedTargets.get(2);

                                cam.grabTargetData(cam.ptt2, 2);

                                temp = cam.tagID[2];

                                cam.tag3 = AprilTagData.getTransform3d(temp);
                            }

                            else {

                                cam.getBestTargetData(cam.plr);
                            }
                        }
                    }
                    try {

                        Thread.sleep(100);

                    } catch (InterruptedException e) {
                        SmartDashboard.putBoolean("OOPS", true);
                    }

                }
            }
        });

        // Set up thread properties and start it off
        grabberThread.setName("PhotonTargetGrabTask");
        grabberThread.setPriority(Thread.MIN_PRIORITY);
        grabberThread.start();
    }

}