package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cameras;
import frc.robot.utils.AprilTagData;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final DriveSubsystem m_drive;

  private final Cameras m_cams;

  // Physical location of the camera on the robot, relative to the center of the
  // robot.
  public final Transform2d CAMERA_TO_ROBOT = new Transform2d(
      new Translation2d(Units.inchesToMeters(12.75), 0.0), new Rotation2d(0.0));

  private final Field2d field2d = new Field2d();

  public PoseEstimatorSubsystem(Cameras cams, DriveSubsystem drive) {
    m_drive = drive;
    m_cams = cams;

  }

  @Override
  public void periodic() {

    if (m_drive.useVisionOdometry) {
      // Update pose estimator with visible targets
      m_cams.plr = m_cams.getLatestResult();

      if (m_cams.plr.hasTargets()) {

        m_cams.targetsAvailable = m_cams.plr.targets.size();

        m_cams.trackedTargets = m_cams.getLatestResult().targets;

        double imageCaptureTime = Timer.getFPGATimestamp() - (m_cams.plr.getLatencyMillis() / 1000d);

        for (int i = 0; i < m_cams.targetsAvailable; i++) {

          m_cams.fiducialID[i] = m_cams.ptt[i].getFiducialId();

          if (AprilTagData.getValidTargetNumber(m_cams.fiducialID[i])) {

            m_cams.ptt[i] = m_cams.trackedTargets.get(i);

            m_cams.tagToCam[i] = m_cams.ptt[i].getCameraToTarget();

            Pose2d targetPose = AprilTagData.getPose2d(m_cams.fiducialID[i]);

            var transform = new Transform2d(
                m_cams.tagToCam[i].getTranslation().toTranslation2d(),
                m_cams.tagToCam[i].getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

            Pose2d camPose = targetPose.transformBy(transform.inverse());

            var visionMeasurement = AprilTagData.getPose2d(m_cams.fiducialID[i]).transformBy(CAMERA_TO_ROBOT);

            field2d.getObject("MyRobot" + m_cams.fiducialID[i]).setPose(visionMeasurement);

            m_drive.m_odometry.addVisionMeasurement(visionMeasurement, imageCaptureTime);
          }
        }
      }
    }

  }

  public Pose2d getCurrentPose() {
    return m_drive.m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    m_drive.resetGyro();
    m_drive.m_odometry.resetPosition(newPose, m_drive.getHeadingRotation2d());
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
