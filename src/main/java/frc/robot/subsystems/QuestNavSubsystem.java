// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  /** Creates a new QuestNavSubsystem. */
  QuestNav questNav = new QuestNav();
  Field2d questField2d = new Field2d();
  Swerve swerveDriveSubsystem = new Swerve();
  Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

  Transform2d robotToQuest = new Transform2d(0, 0, new Rotation2d(0));

  public QuestNavSubsystem() {



  }

  @Override
  public void periodic() {
    if (questNav.isTracking()) {
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Get the pose of the Quest
            Pose2d questPose = questFrame.questPose();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
            Pose2d robotPose = questPose.transformBy(robotToQuest.inverse());

            // You can put some sort of filtering here if you would like!

            // Add the measurement to our estimator
            swerveDriveSubsystem.swerveDrive.addVisionMeasurement(robotPose, timestamp, QUESTNAV_STD_DEVS);
        }
    }
  }


  public void setInitialPose(Pose2d startPose){
    Pose2d questStartPose = startPose.transformBy(robotToQuest.inverse());
    questNav.setPose(questStartPose);
  }
}
