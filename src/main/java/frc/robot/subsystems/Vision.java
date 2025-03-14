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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Photonvision.VisionSim;

public class Vision extends SubsystemBase {

  Swerve swerve ; 
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  // Optional<PhotonCamera> leftCamera;
  // Optional<PhotonCamera> rightCamera;
  // Optional<PhotonCamera> backCamera;
  Optional<PhotonCamera> centerCamera;

  Transform3d centerTagCamera = new Transform3d(new Translation3d(
    Inch.of(-4.625).in(Meters),
    Inch.of(0).in(Meters), 
    Inch.of(17.875).in(Meters)),
    new Rotation3d(0.0, 0.0, Math.toRadians(180.0))
  );
  // Transform3d leftRobotToCam = new Transform3d(new Translation3d(-Inch.of(13.75).in(Meters), Inch.of(11).in(Meters), Inch.of(13.5).in(Meters)), new Rotation3d(0.0, 0.0, Math.toRadians(40.0+180.0)));
  // Transform3d rightRobotToCam = new Transform3d(new Translation3d(-Inch.of(13.75).in(Meters), -Inch.of(11).in(Meters), Inch.of(13.5).in(Meters)), new Rotation3d(0.0, 0.0, -Math.toRadians(40.0+180.0)));
  // Transform3d backRobotToCam = new Transform3d(new Translation3d(), new Rotation3d());
  // PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftRobotToCam);
  // PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightRobotToCam);
  PhotonPoseEstimator centerPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, centerTagCamera);

  Field2d visionField2d = new Field2d();

  Optional<VisionSim> sim = Optional.empty();

  /** Creates a new Vision. */
  public Vision(Swerve swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField2d);

    //move camera constructors here
    
    try{
      centerCamera = Optional.of(new PhotonCamera("Arducam_OV9782_Shooter_Vision"));
      // leftCamera = Optional.empty();
    }catch(Error e){
      System.err.print(e);
      centerCamera = Optional.empty();
    }

    // try{
    //   leftCamera = Optional.of(new PhotonCamera("Back_Left"));
    //   // leftCamera = Optional.empty();
    // }catch(Error e){
    //   System.err.print(e);
    //   leftCamera = Optional.empty();
    // }

    // try{
    //   rightCamera = Optional.of(new PhotonCamera("Back_Right"));
    //   // rightCamera = Optional.empty();
    // }catch(Error e){
    //   System.err.print(e);
    //   rightCamera = Optional.empty();
    // }

    // try{
    //   backCamera = Optional.of(new PhotonCamera("Back"));
    // }catch(Error e){
    //   System.err.print(e);
    //   backCamera = Optional.empty();
    // }

    if(Robot.isSimulation()){ sim = Optional.of(new VisionSim(swerve,leftCamera.get()));}
   
  }

  @Override
  public void periodic() {
    // swerve.getPose()
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("vision/leftCamera", leftCamera.isPresent());
    // SmartDashboard.putBoolean("vision/rightCamera", rightCamera.isPresent());
    // SmartDashboard.putBoolean("vision/backCamera", backCamera.isPresent());
    SmartDashboard.putBoolean("vision/centerCamera", centerCamera.isPresent());

    // if(leftCamera.isPresent()){
    //   PhotonPipelineResult results = leftCamera.get().getLatestResult();
    //   SmartDashboard.putBoolean("vision/seesTarget", results.hasTargets());
    // }

    // updateOdometry();
    //getDistanceFromCamera();

    //SmartDashboard.putNumber("rotation of object", getRotationToObject().orElse(new Rotation2d(-Math.PI)).getDegrees());
  
  }
  
  @Override
  public void simulationPeriodic() {
    if (sim.isPresent()) sim.get().periodic();
  }
  

  public void updateOdometry(){
    //if camera is present
    // if(leftCamera.isPresent()){
    //   updateCameraSideOdometry(leftPoseEstimator, leftCamera.get());
    // }
    // if(rightCamera.isPresent()){
    //   updateCameraSideOdometry(rightPoseEstimator, rightCamera.get());
    // }
    if(centerCamera.isPresent()){
      updateCameraSideOdometry(centerPoseEstimator, centerCamera.get());
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
        //Get some data to help diagnose issues
        double neartestTag=distanceToNearestTag(estimatedPose.get());
        SmartDashboard.putNumber("camera/"+camera.getName()+"/nearestTagDist", neartestTag);
        visionField2d.getObject(camera.getName()).setPose(estimatedPose.get().estimatedPose.toPose2d());


        //Manage std deviations for this result
        Matrix<N3, N1> stddev;
        // stddev = VecBuilder.fill(1, 1, Double.MAX_VALUE).times(5);
        stddev = getStandardDeviationNearest(estimatedPose.get());

        
        swerve.swerveDrive.addVisionMeasurement(
          estimatedPose.get().estimatedPose.toPose2d(),
          result.getTimestampSeconds()//,
          // stddev
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
  

  public Matrix<N3, N1> getStandardDeviationNearest(EstimatedRobotPose estimatedPose){
    //Get one that's easy to work with
    var stddev = VecBuilder.fill(1, 1, Double.MAX_VALUE);

    var targets = estimatedPose.targetsUsed;

    // Avoid using tags with high ambiguity; These generate jitter and bad poses
    for (var tgt : targets){
      if(tgt.poseAmbiguity > 0.2) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    //Why are we even here then?
    if(targets.isEmpty()) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    double poseToTagDistance = 10; //m
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
      poseToTagDistance = Math.min(tag.getDistance(estimate.getTranslation()), poseToTagDistance) ;
    }

    //our base confidence
    var scalar = 1.0;

    var devTagDistance = 1;
    if( poseToTagDistance > 2) devTagDistance*=4;
    if( poseToTagDistance <0.3) devTagDistance*=4;

    return stddev.times(devTagDistance);
    //If we do more conditions, we should consider doing RMS calculations
    // return stddev.times(Math.sqrt(devTagDistance*devTagDistance + devOtherParameter*devOtherParameter));
  }
  
  public double getRotationDouble(){
    var value = getRotationToObject().orElse(new Rotation2d()).rotateBy(swerve.getHeading()).getRotations();
    return value;
  }

  public Optional<Rotation2d> getRotationToObject(){
    // if(backCamera.isPresent()){
    // PhotonPipelineResult results = backCamera.get().getLatestResult();

    // if (results.hasTargets()){
    //   PhotonTrackedTarget target = results.getBestTarget();
    //   return Optional.of(new Rotation2d(Degrees.of(target.getYaw())));
    // }
    // }
    return Optional.empty();
    
  }

}

