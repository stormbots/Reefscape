// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

  
  double maximumSpeed = 5.4;

  SwerveDrive swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public Swerve() {
    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"tabiSwerve");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }  

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      Pounds.of(140), 
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

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public Rotation2d getHeading(){
    return swerveDrive.getGyro().getRotation3d().toRotation2d();
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getChassisSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }
}
