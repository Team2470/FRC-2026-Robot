package frc.robot.subsystems;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestNavConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import frc.robot.Constants;
import frc.robot.VisionMeasurementConsumer;

public class QuestNavSubsystem extends SubsystemBase {

QuestNav questNav = new QuestNav();

private final VisionMeasurementConsumer visionMeasurementConsumer;
public QuestNavSubsystem(
    VisionMeasurementConsumer addVisionMeasurement
){
    this.visionMeasurementConsumer = addVisionMeasurement;
}
    
    @Override
    public void periodic() {
        questNav.commandPeriodic();

        PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();
            // Get the most recent Quest pose

        for (PoseFrame frame : newFrames) {
            if (questNav.isConnected() && questNav.isTracking()) {
            Pose3d questPose = frame.questPose3d();
            double timestamp = frame.dataTimestamp();   
            Pose3d robotPose = questPose.transformBy(new Transform3d(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse()));

            SmartDashboard.putNumber("questNav/Frame x", frame.questPose3d().getX());
            SmartDashboard.putNumber("questNav/Frame y", frame.questPose3d().getY());
            SmartDashboard.putNumber("questNav/Frame z", frame.questPose3d().getZ());
            
            visionMeasurementConsumer.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS); 
            }

        }
    }

    public void setQuestNavPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(new Transform3d(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse()));
        questNav.setPose(questPose);
    }

  /**
   * Sets the QuestNav pose from the given 2D robot pose.
   * <p>
   * This sets the 3D pose with a Z of 0 (on the floor) and no pitch or roll.
   *
   * @param pose2d the robot's 2D pose
   */
  public void setQuestNavPose(Pose2d pose2d) {
    setQuestNavPose(new Pose3d(pose2d));
  }
}
