// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.lang.StackWalker.Option;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  Swerve swerve ; 
  AHRS navx;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  Optional<PhotonCamera> leftCamera;
  Optional<PhotonCamera> rightCamera;
  
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
  PhotonPoseEstimator cameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

  /** Creates a new Vision. */
  public Vision(Swerve swerve, AHRS navxGyro) {

    this.swerve = swerve;
    this.navx = navxGyro;

    //move camera constructors here
    leftCamera = Optional.of(new PhotonCamera("Left"));
    try{

    }catch(Error e){
      System.err.print(e);
      leftCamera = Optional.empty();
    }

    try{
      rightCamera = Optional.of(new PhotonCamera("Right"));
    }catch(Error e){
      System.err.print(e);
      rightCamera = Optional.empty();
    }




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(leftCamera.isPresent()){
      PhotonPipelineResult results = leftCamera.get().getLatestResult();
    

      SmartDashboard.putBoolean("vision/seesTarget", results.hasTargets());
    }

    updateOdometry();
    //getDistanceFromCamera();

    SmartDashboard.putNumber("rotation of object", getRotationToObject().orElse(new Rotation2d(-Math.PI)).getDegrees());
  

  }


  public void updateOdometry(){
    //if camera is present
    if(leftCamera.isPresent()){
      updateCameraSideOdometry(cameraPoseEstimator, leftCamera.get());
    }
  }

  /*public void getDistanceFromCamera(){

    var results = camera.getLatestResult();
    if(results.hasTargets()){
      var pitch = results.getBestTarget().getPitch();   
      SmartDashboard.putNumber("Distance", 
    PhotonUtils.calculateDistanceToTargetMeters(Meters.convertFrom(7.5, Inches), 
    Meters.convertFrom(56, Inches), Radians.convertFrom(30, Degrees), 
    -Radians.convertFrom(pitch, Degrees)));   

    }
  }*/

  private void updateCameraSideOdometry(PhotonPoseEstimator photonPoseEstimator, PhotonCamera camera){

    var latesResults = camera.getAllUnreadResults();
    for(PhotonPipelineResult result : latesResults){
      Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
      SmartDashboard.putBoolean("ispresent", estimatedPose.isPresent());
      if(estimatedPose.isPresent()){  
        double latency;

        if(result.hasTargets()){
          latency = result.getTimestampSeconds() / 1.0e3;
        }
        else{
          latency = 0;
        }
        Matrix<N3, N1> stddev = getStdDeviation(estimatedPose.get());

        SmartDashboard.putNumber("Std deviation", stddev.get(0, 0));
        //update pose

        swerve.swerveDrive.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), result.getTimestampSeconds());
      }
    }
  }

  public Matrix<N3, N1> getStdDeviation(EstimatedRobotPose estimatedPose){

    var estStdDeviation = VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDistance = 0;

    for (var tgt : targets){

      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());

      if(tagPose.isEmpty()) continue;
      numTags++;
      avgDistance += tagPose.get().toPose2d().minus(estimatedPose.estimatedPose.toPose2d()).getTranslation().getNorm();

    }

    if(numTags == 0) return estStdDeviation;
    avgDistance /= numTags;

    if(numTags > 1 && avgDistance > 5){

      estStdDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    } else {

      estStdDeviation = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);

    }

    if(numTags == 1 && avgDistance > 4){

      estStdDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    
    } else {
     
      estStdDeviation = estStdDeviation.times(1 + (avgDistance * avgDistance / 20));

    }

    return estStdDeviation;

  }

  public Optional<Rotation2d> getRotationToObject(){
    if(rightCamera.isPresent()){
    PhotonPipelineResult results = rightCamera.get().getLatestResult();

    if (results.hasTargets()){
      PhotonTrackedTarget target = results.getBestTarget();
      return Optional.of(new Rotation2d(Degrees.of(target.getYaw())));
    }
    }
    return Optional.empty();
    
  }






  


}

