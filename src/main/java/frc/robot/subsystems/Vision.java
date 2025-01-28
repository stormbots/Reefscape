// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  PhotonCamera camera = new PhotonCamera("photonvision");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
  /** Creates a new Vision. */
  public Vision() {
    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //AprilTag+GotoPose system hopefully
    
    var result = camera.getLatestResult();
    if(result.hasTargets()){
      var target = result.getBestTarget();
      Transform3d pose = target.bestCameraToTarget;
      double xtranslation = pose.getX();
      double ytranslation = pose.getY();




    }
  }
}
