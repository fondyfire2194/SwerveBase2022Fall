package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DriveSubsystem drivetrainSubsystem;
  public PhotonPipelineResult plr;
  // Physical location of the camera on the robot, relative to the center of the
  // robot.
  public final Transform2d CAMERA_TO_ROBOT = new Transform2d(
      new Translation2d(Units.inchesToMeters(12.75), 0.0), new Rotation2d(0.0));

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  public final List<Pose2d> targetPoses = Collections.unmodifiableList(List.of(
      new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(39.4375), Rotation2d.fromDegrees(180)),
      new Pose2d(Units.inchesToMeters(84), 0.0, Rotation2d.fromDegrees(180))));

  public final List<Integer> targetPoseIDs = new ArrayList<>();

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  // private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1,
  // Units.degreesToRadians(5));
  // private static final Matrix<N1, N1> localMeasurementStdDevs =
  // VecBuilder.fill(Units.degreesToRadians(0.01));
  // private static final Matrix<N3, N1> visionMeasurementStdDevs =
  // VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  // private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  public int numberOfTargets;

  public int[] fiducialID = { 0, 0 };

  public Pose2d targetPose[] = new Pose2d[2];

  public PhotonTrackedTarget target[] = new PhotonTrackedTarget[2];

  public Transform3d[] camToTarget = new Transform3d[2];
  public Transform2d[] transform = new Transform2d[2];
  public Pose2d[] camPose = new Pose2d[2];

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, DriveSubsystem drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;

    targetPose[0] = new Pose2d();
    targetPose[1] = new Pose2d();
    target[0] = new PhotonTrackedTarget();
    target[1] = new PhotonTrackedTarget();
    camToTarget[0] = new Transform3d();

    transform[0] = new Transform2d();
    transform[1] = new Transform2d();

    targetPoseIDs.add(0);
    targetPoseIDs.add(1);
  }

  @Override
  public void periodic() {

    if (drivetrainSubsystem.useVisionOdometry) {
      // Update pose estimator with visible targets
      plr = photonCamera.getLatestResult();

      SmartDashboard.putBoolean("PLHTG", plr.hasTargets());

      if (plr.hasTargets()) {

        numberOfTargets = plr.targets.size();

        SmartDashboard.putNumber("RTGT", numberOfTargets);

        double imageCaptureTime = Timer.getFPGATimestamp() - (plr.getLatencyMillis() / 1000d);

        for (int i = 0; i < numberOfTargets; i++) {

          target[i] = plr.targets.get(i);

          fiducialID[i] = target[i].getFiducialId();

          if (fiducialID[i] >= 0) {

            targetPose[i] = targetPoses.get(i);

            camToTarget[i] = target[i].getCameraToTarget();

            transform[i] = new Transform2d(

                camToTarget[i].getTranslation().toTranslation2d(),

                camToTarget[i].getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

            camPose[i] = targetPose[i].transformBy(transform[i].inverse());

            var visionMeasurement = camPose[1].transformBy(CAMERA_TO_ROBOT);

            field2d.getObject("MyRobot" + fiducialID).setPose(visionMeasurement);

            drivetrainSubsystem.m_odometry.addVisionMeasurement(visionMeasurement, imageCaptureTime);
          }
        }

      }

    }
  }

  public Pose2d getCurrentPose() {
    return drivetrainSubsystem.m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    drivetrainSubsystem.resetGyro();
    drivetrainSubsystem.m_odometry.resetPosition(newPose, drivetrainSubsystem.getHeadingRotation2d());
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    drivetrainSubsystem.resetGyro();
    drivetrainSubsystem.m_odometry.resetPosition(
        new Pose2d(), drivetrainSubsystem.getHeadingRotation2d());
  }

}
