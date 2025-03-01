// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.deser.SettableBeanProperty.Delegating;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldNavigation;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

  public Field2d debugField2d = new Field2d();
  public Field2d odometryField = new Field2d();
  private FieldNavigation fieldNav = new FieldNavigation();

  
  double maximumSpeed = 5.033;

  SwerveDrive swerveDrive;

  PathConstraints constraintsFast = new PathConstraints(5, 3.5, 5, 3);
  PathConstraints constraintsSlow = new PathConstraints(2.5, 1.75, 5, 1.5);

  /** Creates a new SwerveSubsystem. */
  public Swerve() {

    SmartDashboard.putData("SwerveDebugField",debugField2d);


    

    
    //Need to turn this back on when running path, commented out for now because its angry
    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"skipper");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }  

    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0.1, 2.4, 0.35));
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));

    swerveDrive.setMotorIdleMode(true); //just to be safe
    configurePathplanner();
    // PathfindingCommand.warmupCommand();

    new Trigger(DriverStation::isTeleopEnabled).onTrue(new InstantCommand(()->
      swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0.1, 2.4, 0))
    ));

    new Trigger(DriverStation::isAutonomousEnabled).onTrue(new InstantCommand(()->
      swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0.1, 2.4, 0.35))
    ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
    debugField2d.setRobotPose(swerveDrive.getPose());
    odometryField.setRobotPose(swerveDrive.getPose());
    
    SmartDashboard.putNumber("heading", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putData("odometryField", odometryField);
    Logger.recordOutput("SwerveModuleStates", swerveDrive.getStates());
    Logger.recordOutput("swerve/pose", swerveDrive.getPose());
    SmartDashboard.putNumber("swerve/x", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("swerve/y", swerveDrive.getPose().getY());

  }

  public void configurePathplanner(){
    RobotConfig robotConfig = null;

    // double kModuleXOffset = Meters.convertFrom(23.5, Inches)/2.0;
    // double kModuleYOffset = Meters.convertFrom(23.5, Inches)/2.0;


    // Translation2d[] moduleOffsets = new Translation2d[]{
    //   new Translation2d(kModuleXOffset, kModuleYOffset),
    //   new Translation2d(kModuleXOffset, -kModuleYOffset),
    //   new Translation2d(-kModuleXOffset, kModuleYOffset),
    //   new Translation2d(-kModuleXOffset, -kModuleYOffset)
    // };

    // ModuleConfig moduleConfig = new ModuleConfig(
    //   Meters.convertFrom(1.5, Inches), 
    //   5.1, 
    //   1.3, 
    //   DCMotor.getNeoVortex(1).withReduction(5.076923076923077), 
    //   40, 
    //   1);

    // robotConfig = new RobotConfig(
    //   Pounds.of(30), 
    //   KilogramSquareMeters.of(15), 
    //   moduleConfig, 
    //   moduleOffsets
    // );

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // TODO: handle exception
    }

    BooleanSupplier shouldFlipPath = () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getChassisSpeeds,
      this::setChassisSpeeds, 
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
      ), 
      robotConfig, 
      shouldFlipPath, 
      this
    );

  
  }

  public Command driveCommandAllianceManaged(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    BooleanSupplier isRed = () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    return run(() -> {
      // Make the robot move
      if(isRed.getAsBoolean()){
        swerveDrive.drive(new Translation2d(-translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      }
      else{
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      }
    });
  }

  private Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  //Robot Relative version
  /**
   * 
   * @param translationX
   * @param translationY 
   * @param angularRotationX Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot relativity.
   * @return
   */
  public Command driveCommandRobotRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  public Command driveAlignedToHeading(DoubleSupplier translationX, DoubleSupplier translationY, Rotation2d desiredRotation){
    PIDController pid = new PIDController(2.0/360, 0.0, 0.0);
    return run(()->{
      double power = pid.calculate(getPose().getRotation().getDegrees(), desiredRotation.getDegrees());
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        power * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  public Rotation2d getHeading(){
    return swerveDrive.getGyro().getRotation3d().toRotation2d();
  }


  public Command goToPose(Pose2d targetPoseIgnore){

    return run(()->{
      //get current pose
      var pose = getPose();
      List<Pose2d> list = new ArrayList<>();{{
        // add(new Pose2d(1,2,new Rotation2d())); //how to add to a fixed object
      }};
      
  
      list.add(debugField2d.getObject("targetPoseOne").getPose());
      list.add(debugField2d.getObject("targetPoseTwo").getPose());
      list.add(debugField2d.getObject("targetPoseThree").getPose());

      var targetPose = pose.nearest(list);

      var delta = targetPose.relativeTo(pose);  
      //THIS DON'T WORK YET, but trying to do pathfinding
     // PathConstraints constraints = new PathConstraints(5, 3.5, 5, 3);
      
     // AutoBuilder.pathfindToPose(targetPose, constraints);

      // delta.getTranslation().getNorm(); //distance to target pose


      var distancex_m = delta.getMeasureX().in(Units.Meter); // the distance in the x axis
      var distancey_m = delta.getMeasureY().in(Units.Meter); //distnace in y
      var rotation_d = delta.getRotation().getDegrees();



      debugField2d.getRobotObject().setPose(pose);
      debugField2d.getObject("targetPose").setPose(targetPose);
      debugField2d.getObject("delta").setPose(delta);
      debugField2d.getObject("targetPoses").setPoses(list);

      //move -> robot relative
      swerveDrive.drive(new Translation2d(distancex_m * swerveDrive.getMaximumChassisVelocity(),
                                          distancey_m * swerveDrive.getMaximumChassisVelocity()),
                        (rotation_d/360.0) * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);

    });
  }

  //does not reset based off field
  private void pidToPose(Pose2d pose){
    final double transltionP = 3.0;
    final double thetaP = 2.0*4
    ;

    // Transform2d delta = pose.minus(swerveDrive.getPose());

    // swerveDrive.setChassisSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(
    //   -delta.getX()*transltionP, 
    //   -delta.getY()*transltionP, 
    //   -delta.getRotation().getRadians()*thetaP
    //   ), swerveDrive.getOdometryHeading()
    // ));

    Pose2d delta = pose.relativeTo(swerveDrive.getPose());

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(
      delta.getX()*transltionP,
      delta.getY()*transltionP,
      delta.getRotation().getRadians()*thetaP
    ));
  }

  public Command pidToPoseCommand(Pose2d poseSupplier){
    return run(()->pidToPose(poseSupplier));
  }

  private void pidToPoseOdometryManaged(Pose2d pose){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Red){
      //Flips across y=x, not across vertical acis
      pose = new Pose2d(17.55-pose.getX(),8.05-pose.getY(), pose.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));
    }

    pidToPose(pose);
  }

  private Command privatePathToPose(Pose2d pose){
    return AutoBuilder.pathfindToPose(pose, constraintsSlow);
  }

  public Command followPath(PathPlannerPath path)
  {
    return AutoBuilder.followPath(path);
  }

  // public Command pathToCoralLeft(){
  //   Set<Subsystem> set = Set.of(this);
  //   return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralLeft(getPose())),set);
  // }

  public Command pathToCoralLeft(){
    return new DeferredCommand(()->pidToPoseCommand(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }
  // public Command pathToCoralRight(){
  //   Set<Subsystem> set = Set.of(this);
  //   return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralRight(getPose())),set);
  // }

  public Command pathToCoralRight(){
    return new DeferredCommand(()->pidToPoseCommand(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command bruh(){
    return new DeferredCommand(()->pidToPoseCommand(new Pose2d(2,7,new Rotation2d())), Set.of(this));
  }

  public Command pathToReefAlgae(){
    Set<Subsystem> set = Set.of(this);
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getReefAlgae(getPose())),set);
  }
  


  public Command pathToPath(PathPlannerPath targetPath){
    //robot constraints for pathPlanner
    PathConstraints constraints = new PathConstraints(5, 3.5, 5, 3);

    //Go to target Path and follow it
    return AutoBuilder.pathfindThenFollowPath(targetPath, constraints);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  private void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void resetOdometryAllianceManaged(Pose2d initialHolonomicPose){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if(alliance == Alliance.Red){
      //Flips across y=x, not across vertical acis
      initialHolonomicPose  = new Pose2d(17.55-initialHolonomicPose.getX(),8.05-initialHolonomicPose.getY(), initialHolonomicPose.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));
    }
    
    resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  // do we need the function based version?
   public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
     swerveDrive.setChassisSpeeds(chassisSpeeds);
   }
  
  public Command resetGyro(){
    return runOnce(swerveDrive::zeroGyro).ignoringDisable(true);
  }

  public Command stop(){
    return new InstantCommand(()->swerveDrive.setChassisSpeeds(new ChassisSpeeds()));
  }

  public Command followPath(String name){
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name)).andThen(stop());
    } catch (Exception e) {
      // TODO: handle exception
      for(int i=0; i<1000; i++){
        System.out.println(e);
      }
      return new InstantCommand();
    }
  }
}
