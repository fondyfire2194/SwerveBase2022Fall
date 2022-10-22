package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT_3D;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPoseEstimatorSubsystem extends SubsystemBase {

  private final DriveSubsystem m_drive;

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  public static final List<Pose3d> targetPoses =

      Collections.unmodifiableList(

          List.of(

              new Pose3d(3.0, .566, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),

              new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0))) ,
              
              new Pose3d(3.0, 1.167, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),

              new Pose3d(3.0, 0.01, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0))),

               new Pose3d(3.0, 1.166, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),

              new Pose3d(3.0, 0.02, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))           
              
              
              
              ));

  public PhotonCamera m_phCam = new PhotonCamera("camera");

  private PhotonPipelineResult previousPipelineResult = null;

  public static Pose3d visionMeasurement = new Pose3d();

  public static Pose3d camPose = new Pose3d();

  public static Transform3d camToTarget = new Transform3d();

  public VisionPoseEstimatorSubsystem(DriveSubsystem drive) {

    m_drive = drive;

  }

  @Override
  public void periodic() {

    // Update pose estimator with visible targets
    var pipelineResult = m_phCam.getLatestResult();

    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {

      previousPipelineResult = pipelineResult;

      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {

        var fiducialId = target.getFiducialId();

        if (fiducialId >= 0 && fiducialId < 10) {

          var targetPose = targetPoses.get(fiducialId);

          SmartDashboard.putString("TargetPoseFound", targetPose.toString());

          camToTarget = target.getCameraToTarget();

          // Workaround until PhotonVision changes Rotation
          camToTarget = camToTarget.plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, -Math.PI / 2)));

          camPose = targetPose.transformBy(camToTarget.inverse());

          visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT_3D);

         // m_drive.m_odometry.addVisionMeasurement(visionMeasurement.toPose2d(), imageCaptureTime);
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
