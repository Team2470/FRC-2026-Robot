package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.Localization.Vision;


public class ResetPoseCommand extends Command{
    private final Vision limelight;
    private final QuestNavSubsystem quest;

    public ResetPoseCommand(Vision    limelight,
                         QuestNavSubsystem        quest){

        this.limelight    = limelight;
        this.quest      = quest;
        addRequirements(limelight, quest);
    }

    @Override
    public void execute(){
        Pose2d limelightPose = new Pose2d(limelight.poseX, limelight.poseY, new Rotation2d(limelight.Rotation));
        quest.setQuestNavPose(limelightPose);
    }
}
