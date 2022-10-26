package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT_3D;

import java.util.Collections;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cameras;

public class VisionPoseEstimatorSubsystem extends SubsystemBase {

  private final DriveSubsystem m_drive;

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  public final List<Pose3d> targetPoses =

      Collections.unmodifiableList(

          List.of(

              new Pose3d(3.0, .566, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),

              new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0))),

              new Pose3d(12, 5, .5, new Rotation3d(0, 0, degreesToRadians(90))),

              new Pose3d(3.0, 0.01, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0))),

              new Pose3d(12.0, 3.0, .5, new Rotation3d(0, 0, degreesToRadians(90.0))),

              new Pose3d(9., 4.0, 0, new Rotation3d(0, 0, degreesToRadians(10)))

          ));

  private PhotonPipelineResult previousPipelineResult = null;

  public Pose3d[] targetPose = new Pose3d[3];

  public Pose3d[] visionMeasurement = new Pose3d[3];

  public Pose3d[] camPose = new Pose3d[3];

  public Pose2d[] temp = new Pose2d[3];

  public Transform3d[] camToTarget = new Transform3d[3];

  public int[] fiducialId = { 0, 0, 0 };

  public int n;

  public VisionPoseEstimatorSubsystem(DriveSubsystem drive) {

    m_drive = drive;

    for (int i = 0; i < 2; i++) {
      camPose[i] = new Pose3d();
      targetPose[i] = new Pose3d();
      temp[i] = new Pose2d();
      camToTarget[i] = new Transform3d();
      visionMeasurement[i] = new Pose3d();
      temp[i] = new Pose2d();

    }

  }

  @Override
  public void periodic() {

    // Update pose estimator with visible targets
    var pipelineResult = Cameras.llcam.getLatestResult();

    SmartDashboard.putBoolean("HASTARGETS", pipelineResult.hasTargets());

    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {

      previousPipelineResult = pipelineResult;

      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);

      SmartDashboard.putNumber("ImCapTime", imageCaptureTime);

      SmartDashboard.putNumber("TargetsSeen", pipelineResult.targets.size());    

      n = 0;

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {

        fiducialId[n] = target.getFiducialId();

        SmartDashboard.putNumber("FidID " + String.valueOf(n), fiducialId[n]);

        if (fiducialId[n] >= 0 && fiducialId[n] < 10) {

          targetPose[n] = targetPoses.get(fiducialId[n]);

          SmartDashboard.putString("TargetPoseFound " + String.valueOf(n), targetPose[n].toString());

          camToTarget[n] = target.getCameraToTarget();

          // Workaround until PhotonVision changes Rotation
          camToTarget[n] = camToTarget[n].plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2)));

          camPose[n] = targetPose[n].transformBy(camToTarget[n].inverse());

          double tempx = targetPose[n].getX() - camToTarget[n].getX();
          double tempy = targetPose[n].getY() - camToTarget[n].getY();
          double tempA = targetPose[n].getRotation().getAngle();

          temp[n] = new Pose2d(tempx, tempy, new Rotation2d(tempA));

          visionMeasurement[n] = camPose[n].transformBy(CAMERA_TO_ROBOT_3D);

          if (m_drive.useVisionOdometry)

            m_drive.m_odometry.addVisionMeasurement(visionMeasurement[n].toPose2d(),
                imageCaptureTime);

          n++;
        }
      }
    }

  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)",
        Units.metersToInches(pose.getX()),
        Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    return m_drive.m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    m_drive.resetGyro();
    m_drive.m_odometry.resetPosition(
        new Pose2d(), m_drive.getHeadingRotation2d());
  }

}
