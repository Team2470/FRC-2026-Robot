package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem {

QuestNav questNav = new QuestNav();

    
@Override
public void periodic() {

    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    
    for (PoseFrame questFrame : questFrames) {
    
        if (questFrame.isTracking()) {
           
            Pose3d questPose = questFrame.questPose3d();
            
            double timestamp = questFrame.dataTimestamp();

            Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
        }
    }
}
}
