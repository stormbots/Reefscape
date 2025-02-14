// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Add your docs here. */
public class FieldNavigation {

    public static List<Pose2d> tagsReef = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(6).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(7).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(8).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(9).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(10).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(11).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(17).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(19).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(20).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(22).get().toPose2d());
    }};
    public static List<Pose2d> tagsSource = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(1).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(2).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(13).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(12).get().toPose2d());
    }};
    public static List<Pose2d> tagsProcessor = new ArrayList<>(){{
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(3).get().toPose2d());
        add(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(16).get().toPose2d());
    }};

    public Field2d field = new Field2d();

    public static Pose2d blueBar = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(14).get().toPose2d();
    public static Pose2d redBar = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(14).get().toPose2d();


    public static Pose2d getNearestReef(Pose2d currentPose)
    {
        return currentPose.nearest(tagsReef);
    }
    public static Pose2d getNearestSource(Pose2d currentPose)
    {
        return currentPose.nearest(tagsSource);
    }
    public static Pose2d getNearestProcessor(Pose2d currentPose)
    {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        return currentPose.nearest(tagsProcessor);
    }
    public Pose2d getTransform(Pose2d poseMoved)
    {
        //This is WIP, forgot to set up a bunch of stuff
       var tag = field.getObject("tag").getPose();

        field.getObject("poseRight").setPose(tag.transformBy(new Transform2d(new Pose2d(), poseMoved)));
        return new Pose2d();

    }

}
