// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Cameras;

/** Add your docs here. */
public class VisionTargetGrabber {
    public volatile boolean isStopped = false;
    int j;

    public VisionTargetGrabber(Cameras cam) {

        Thread grabberThread = new Thread(new Runnable() {
            int t;
            int temp;

            @Override
            public void run() {

                while (!isStopped) {

                    SmartDashboard.putNumber("THRNG", t++);

                    cam.plr = cam.getLatestResult();

                    cam.hasTargets = cam.getHasTargets(cam.plr);

                    if (!cam.hasTargets)
                        cam.targetsAvailable = 0;

                    if (cam.hasTargets) {
                        cam.targetsAvailable = cam.plr.targets.size();

                        temp = cam.plr.getBestTarget().getFiducialId();

                        SmartDashboard.putNumber("TTTEE", temp);

                        SmartDashboard.putNumber("OOO", temp);

                        cam.latencySeconds = cam.getLatencySeconds(cam.plr);

                        if (AprilTagData.getValidTargetNumber(temp)) {

                            cam.targetLocationNames[0] = AprilTagData.getTagLocation(temp);

                            cam.ptt0 = cam.plr.getBestTarget();

                            cam.grabTargetData(cam.ptt0, 0);

                        }
                        // look for second target

                        if (!cam.bestTargetOnly) {

                            cam.trackedTargets = cam.getTrackedTargets(cam.plr);

                            cam.targetsAvailable = cam.trackedTargets.size();

                            if (cam.targetsAvailable > 1) {

                                cam.ptt1 = cam.trackedTargets.get(1);

                                temp = cam.ptt1.getFiducialId();

                                if (AprilTagData.getValidTargetNumber(temp)) {

                                    cam.targetLocationNames[1] = AprilTagData.getTagLocation(temp);

                                    cam.grabTargetData(cam.ptt1, 1);

                                }
                            }

                            if (cam.targetsAvailable >= 2) {

                                cam.ptt2 = cam.trackedTargets.get(2);

                                temp = cam.ptt2.getFiducialId();

                                if (AprilTagData.getValidTargetNumber(temp)) {

                                    cam.targetLocationNames[temp] = AprilTagData.getTagLocation(temp);

                                    cam.grabTargetData(cam.ptt2, 2);

                                }

                            }

                        } else {

                            SmartDashboard.putNumber("nnn", 911);
                        }
                    } else {
                        cam.clear2dResults();
                        cam.clear3dResults();
                        cam.targetsAvailable = 0;
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
