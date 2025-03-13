// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.stormbots.Lerp;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  Swerve swerve ; 
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  Optional<PhotonCamera> leftCamera;
  Optional<PhotonCamera> rightCamera;
  Optional<PhotonCamera> backCamera;

  Transform3d leftRobotToCam = new Transform3d(new Translation3d(-Inch.of(0).in(Meters), Inch.of(0).in(Meters), Inch.of(0).in(Meters)), new Rotation3d(0.0, 0.0, Math.toRadians(40.0+180.0)));
  Transform3d rightRobotToCam = new Transform3d(new Translation3d(-Inch.of(13.75).in(Meters), -Inch.of(11).in(Meters), Inch.of(13.5).in(Meters)), new Rotation3d(0.0, 0.0, -Math.toRadians(40.0+180.0)));
  Transform3d backRobotToCam = new Transform3d(new Translation3d(), new Rotation3d());
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftRobotToCam);
  PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightRobotToCam);
  
  Field2d visionField2d = new Field2d();
   
  /** Creates a new Vision. */
  public Vision(Swerve swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField2d);

    //move camera constructors here
    
    try{
      leftCamera = Optional.of(new PhotonCamera("Back_Left"));
      // leftCamera = Optional.empty();
    }catch(Error e){
      System.err.print(e);
      leftCamera = Optional.empty();
    }

    try{
      rightCamera = Optional.of(new PhotonCamera("Back_Right"));
      // rightCamera = Optional.empty();
    }catch(Error e){
      System.err.print(e);
      rightCamera = Optional.empty();
    }

    try{
      backCamera = Optional.of(new PhotonCamera("Back"));
    }catch(Error e){
      System.err.print(e);
      backCamera = Optional.empty();
    }


  }

  @Override
  public void periodic() {
    // swerve.getPose()
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("vision/leftCamera", leftCamera.isPresent());
    SmartDashboard.putBoolean("vision/rightCamera", rightCamera.isPresent());
    SmartDashboard.putBoolean("vision/backCamera", backCamera.isPresent());

    // if(leftCamera.isPresent()){
    //   PhotonPipelineResult results = leftCamera.get().getLatestResult();
    //   SmartDashboard.putBoolean("vision/seesTarget", results.hasTargets());
    // }

    updateOdometry();
    //getDistanceFromCamera();

    //SmartDashboard.putNumber("rotation of object", getRotationToObject().orElse(new Rotation2d(-Math.PI)).getDegrees());
  
  }


  public void updateOdometry(){
    //if camera is present
    if(leftCamera.isPresent()){
      updateCameraSideOdometry(leftPoseEstimator, leftCamera.get());
    }
    if(rightCamera.isPresent()){
      updateCameraSideOdometry(rightPoseEstimator, rightCamera.get());
    }
  }
  // public void getDistanceFromCamera(){

  //   if(leftCamera.isPresent()){
  //   var results = leftCamera.get().getAllUnreadResults();
  //   // if(results.hasTargets()){
  //     // var pitch = results.getBestTarget().getPitch();   
  //     // SmartDashboard.putNumber("Distance", 0);
  //   //   PhotonUtils.calculateDistanceToTargetMeters(
  //   //     Meters.convertFrom(7.5, Inches), 
  //   //     Meters.convertFrom(56, Inches),
  //   //     Radians.convertFrom(30, Degrees), 
  //   //     -Radians.convertFrom(pitch, Degrees))
  //   // );   
  //   }

  //   }
  // }

  private void updateCameraSideOdometry(PhotonPoseEstimator photonPoseEstimator, PhotonCamera camera){

    var latesResults = camera.getAllUnreadResults();
    for(PhotonPipelineResult result : latesResults){
      Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
      // SmartDashboard.putBoolean("ispresent", estimatedPose.isPresent());
      if(estimatedPose.isPresent()){  
        
        Matrix<N3, N1> stddev;
        stddev = VecBuilder.fill(1, 1, Double.MAX_VALUE).times(5);

        // stddev = getStandardDeviationNearest(estimatedPose.get());

        double neartestTag=distanceToNearestTag(estimatedPose.get());
        // if(neartestTag<0.2) stddev = stddev.times(2);
        SmartDashboard.putNumber("camera/"+camera.getName()+"/nearestTagDist", neartestTag);
        visionField2d.getObject(camera.getName()).setPose(estimatedPose.get().estimatedPose.toPose2d());

        // swerve.swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
        swerve.swerveDrive.addVisionMeasurement(
          estimatedPose.get().estimatedPose.toPose2d(),
          result.getTimestampSeconds(),
          stddev
        );
      }
    }
  }

  public double distanceToNearestTag(EstimatedRobotPose estimatedPose){
    var targets = estimatedPose.targetsUsed;
    double distanceToEstimatedPose = 10; //m
    var estimate = estimatedPose.estimatedPose.toPose2d();

    for (var tgt : targets){
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if(tagPose.isEmpty()) continue;

      var tag = tagPose.get().toPose2d().getTranslation();
      distanceToEstimatedPose = Math.min(tag.getDistance(estimate.getTranslation()), distanceToEstimatedPose) ;
    }

    var bot = swerve.getPose().getTranslation();

    return Math.min(bot.getDistance(estimate.getTranslation()), distanceToEstimatedPose);
  }


  public Matrix<N3, N1> getStdDeviation(EstimatedRobotPose estimatedPose){

    var estStdDeviation = VecBuilder.fill(5, 5, Double.MAX_VALUE);
    var targets = estimatedPose.targetsUsed;
    int numTags = 0;
    double avgDistance = 0;

    //Generate the average distance to all seen tags
    for (var tgt : targets){
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if(tagPose.isEmpty()) continue;
      numTags++;
      avgDistance += tagPose.get().toPose2d().minus(estimatedPose.estimatedPose.toPose2d()).getTranslation().getNorm();
    }
    //If no tags were useful for positions, avoid divide by zero; We shouldn't be checking stdevs if no targets exist
    if(numTags == 0) return estStdDeviation;
    avgDistance /= numTags;

    //If estimated pose is far away from tags, it's invalid
    if(numTags > 1 && avgDistance > 5){
      estStdDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else { // multiple tags, and estimated pose is within viable ranges
      estStdDeviation = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    }

    //If we see exactly one tag and it's saying we're far away, ignore it
    if(numTags == 1 && avgDistance > 4){
      estStdDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else { //see multiple tags and/or they're close.
      //This is doing *all* the heavy lifting here, but the best case is still a stdev of 0.5m (dist=0)
      // Worst case on last year's field is 0.9
      //This year, it'd be ~0.6. This is not exactly *doing much* to actually change the stdev. 
      estStdDeviation = estStdDeviation.times(1 + (avgDistance * avgDistance / 20));
    }
    estStdDeviation = estStdDeviation.times(2);
    return estStdDeviation;
  }


  public Matrix<N3, N1> getStandardDeviationNearest(EstimatedRobotPose estimatedPose){
    //Get one that's easy to work with
    var stddev = VecBuilder.fill(1, 1, Double.MAX_VALUE);

    var targets = estimatedPose.targetsUsed;
    if(targets.isEmpty()) return stddev.times(10);

    double distanceToEstimatedPose = 10; //m
    double distanceToCurrentPose = 10; //m
    var estimate = estimatedPose.estimatedPose.toPose2d();
    var bot = swerve.getPose().getTranslation();

    //See how far away we're being told to move
    distanceToCurrentPose = Math.min(bot.getDistance(estimate.getTranslation()), distanceToCurrentPose);

    //Check how far away the tag is from where the tags say we *should* be.
    for (var tgt : targets){
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if(tagPose.isEmpty()) continue;

      var tag = tagPose.get().toPose2d().getTranslation();
      distanceToEstimatedPose = Math.min(tag.getDistance(estimate.getTranslation()), distanceToEstimatedPose) ;
    }

    var scalar = 1.0;

    //if we're seeing tags near the estimated pose, decrease stdev ; That means we're close to it. 
    distanceToEstimatedPose = MathUtil.clamp(distanceToEstimatedPose,0, 5);
    scalar *= Lerp.lerp(distanceToEstimatedPose, 0.1, 4, 1, 4);

    // if we're being told to move a long distance, increase stdev; That means something went wrong at some point
    scalar *= Lerp.lerp(distanceToCurrentPose, 0, 10, 1, 10);

    //if we're moving fast, increase stdev; Speed introduces potential errors
    var speedsobject = swerve.getChassisSpeeds();
    var speed = Math.hypot(speedsobject.vxMetersPerSecond,speedsobject.vyMetersPerSecond);
    var omega = speedsobject.omegaRadiansPerSecond;
    scalar *= Lerp.lerp(speed, 0, 5, 1, 3);
    //Fast rotations are particularly bad; Negate this harshly
    scalar *= Lerp.lerp(omega, 0, Math.PI, 1, 10);

    //Sanity check to make sure our reported errors are where we'd kind of expect; Between a couple inches and "on the field"
    scalar = MathUtil.clamp(scalar,2, 40);

    SmartDashboard.putNumber("vision/stdev",scalar);
    return stddev.times(scalar);
  }

  

  public Matrix<N3, N1> getStdevTake3(EstimatedRobotPose estimatedPose){
    //Get one that's easy to work with
    var stddev = VecBuilder.fill(1, 1, Double.MAX_VALUE);

    var targets = estimatedPose.targetsUsed;
    if(targets.isEmpty()) return stddev.times(10);

    double distanceToEstimatedPose = 10; //m
    double distanceToCurrentPose = 10; //m
    var estimate = estimatedPose.estimatedPose.toPose2d();
    var bot = swerve.getPose().getTranslation();

    //See how far away we're being told to move
    distanceToCurrentPose = Math.min(bot.getDistance(estimate.getTranslation()), distanceToCurrentPose);

    //Check how far away the tag is from where the tags say we *should* be.
    for (var tgt : targets){
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if(tagPose.isEmpty()) continue;

      var tag = tagPose.get().toPose2d().getTranslation();
      distanceToEstimatedPose = Math.min(tag.getDistance(estimate.getTranslation()), distanceToEstimatedPose) ;
    }

    var scalar = 1.0;

    //if we're seeing tags near the estimated pose, decrease stdev ; That means we're close to it. 
    distanceToEstimatedPose = MathUtil.clamp(distanceToEstimatedPose,0, 5);
    scalar *= Lerp.lerp(distanceToEstimatedPose, 0.1, 4, 0.1, 4);


    // if we're being told to move a long distance, increase stdev; That means something went wrong at some point
    scalar *= Lerp.lerp(distanceToCurrentPose, 0, 10, 0.1, 10);

    //if we're moving fast, increase stdev; Speed introduces potential errors
    var speedsobject = swerve.getChassisSpeeds();
    var speed = Math.hypot(speedsobject.vxMetersPerSecond,speedsobject.vyMetersPerSecond);
    var omega = speedsobject.omegaRadiansPerSecond;
    scalar *= Lerp.lerp(speed, 0, 5, 1, 3);
    //Fast rotations are particularly bad; Negate this harshly
    scalar *= Lerp.lerp(omega, 0, Math.PI, 1, 10);

    //Sanity check to make sure our reported errors are where we'd kind of expect; Between a couple inches and "on the field"
    scalar = MathUtil.clamp(scalar,1, 30);
    return stddev.times(scalar);
  }


  public double getRotationDouble(){
    var value = getRotationToObject().orElse(new Rotation2d()).rotateBy(swerve.getHeading()).getRotations();
    return value;
  }

  public Optional<Rotation2d> getRotationToObject(){
    if(backCamera.isPresent()){
    PhotonPipelineResult results = backCamera.get().getLatestResult();

    if (results.hasTargets()){
      PhotonTrackedTarget target = results.getBestTarget();
      return Optional.of(new Rotation2d(Degrees.of(target.getYaw())));
    }
    }
    return Optional.empty();
    
  }

}

