// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

  public Field2d debugField2d = new Field2d();
  public Field2d odometryField = new Field2d();
  
  double maximumSpeed = 5.4;

  SwerveDrive swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public Swerve() {

    SmartDashboard.putData("SwerveDebugField",debugField2d);
    debugField2d.getObject("targetPoseOne").setPose(new Pose2d(3.5, 3.1, new Rotation2d(0.0)));
    debugField2d.getObject("targetPoseTwo").setPose(new Pose2d(4.0, 5.25, new Rotation2d(0.5)));
    debugField2d.getObject("targetPoseThree").setPose(new Pose2d(6.3, 4.1, new Rotation2d(0.5)));
    
    //Need to turn this back on when running path, commented out for now because its angry
    //configurePathplanner();
    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"practiceBot");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }  

    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0.080663, 1.9711, 0.36785));
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
    odometryField.setRobotPose(swerveDrive.getPose());
    SmartDashboard.putData("odometryField", odometryField);
    Logger.recordOutput("SwerveModuleStates", swerveDrive.getStates());

  }

  public void configurePathplanner(){
    RobotConfig robotConfig;

    double kModuleXOffset = Meters.convertFrom(23.5, Inches)/2.0;
    double kModuleYOffset = Meters.convertFrom(23.5, Inches)/2.0;


    Translation2d[] moduleOffsets = new Translation2d[]{
      new Translation2d(kModuleXOffset, kModuleYOffset),
      new Translation2d(kModuleXOffset, -kModuleYOffset),
      new Translation2d(-kModuleXOffset, kModuleYOffset),
      new Translation2d(-kModuleXOffset, -kModuleYOffset)
    };

    ModuleConfig moduleConfig = new ModuleConfig(
      3, 
      5.1, 
      1.3, 
      DCMotor.getNeoVortex(1).withReduction(5.076923076923077), 
      40, 
      1);

    robotConfig = new RobotConfig(
      Pounds.of(30), 
      KilogramSquareMeters.of(15), 
      moduleConfig, 
      moduleOffsets
    );

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


  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
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

       var gain = pose.interpolate(targetPose, 0.5);

      var distancex_m = delta.getMeasureX().in(Units.Meter); // the distance in the x axis
      var distancey_m = delta.getMeasureY().in(Units.Meter); //distnace in y
      var rotation_d = delta.getRotation().getDegrees();

      var outx = distancex_m * gain.getX();
      var outy = distancey_m * gain.getY();
      var outRotation = (rotation_d/360.0)*(gain.getRotation().getDegrees()/360);


      debugField2d.getRobotObject().setPose(pose);
      debugField2d.getObject("targetPose").setPose(targetPose);
      debugField2d.getObject("delta").setPose(delta);
      debugField2d.getObject("targetPoses").setPoses(list);

      //move -> robot relative
      swerveDrive.drive(new Translation2d(outx * swerveDrive.getMaximumChassisVelocity(),
                                          outy * swerveDrive.getMaximumChassisVelocity()),
                        outRotation * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);

    });
  }

  public Command pathToPose(Pose2d targetPoseIgnore){
    var pose = getPose();
    List<Pose2d> list = new ArrayList<>();{{
      // add(new Pose2d(1,2,new Rotation2d())); //how to add to a fixed object
    }};

    list.add(debugField2d.getObject("targetPoseOne").getPose());
    list.add(debugField2d.getObject("targetPoseTwo").getPose());
    list.add(debugField2d.getObject("targetPoseThree").getPose());
    
    var targetPose = pose.nearest(list);
    PathConstraints constraints = new PathConstraints(5, 3.5, 5, 3);
    return AutoBuilder.pathfindToPose(targetPose, constraints, Units.MetersPerSecond.of(0.5));
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
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

  //public Command driveToPose(){
   // 
  //}
}
