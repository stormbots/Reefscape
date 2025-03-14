// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

/** Add your docs here. */
public class ElevatorMotorConfigs {
  public static SparkFlexConfig armConfig;
  public static SparkFlexConfig scorerConfig;

  public static SparkBaseConfig getElevatorConfig(){
    SparkBaseConfig elevatorConfig = new SparkMaxConfig()
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      ;
    elevatorConfig.softLimit
    .forwardSoftLimit(33.5)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(0)
    .reverseSoftLimitEnabled(true);
    ;
    //.18 measured, 65.25 actual
    var elevatorConversionfactor =(33.5/74.4);//(72.375-8.5)/(52.9+18.1);
    elevatorConfig.encoder
        .positionConversionFactor(elevatorConversionfactor)
        .velocityConversionFactor(elevatorConversionfactor / 60.0)
        ;
    elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1/12.0*2*2*1.5);

    return elevatorConfig;
  }

  public static SparkBaseConfig getRotationConfig(){
    SparkBaseConfig rotationConfig = new SparkMaxConfig()
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      ;

    rotationConfig.softLimit
      .reverseSoftLimit(-90)
      .forwardSoftLimit(170)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false)
      ;

    //90 degrees is 18.8 rotations 
    double rotateCoversionFactor = 1/25.0*(18.0/64.0) * 360;
    rotationConfig.encoder
        .velocityConversionFactor(rotateCoversionFactor / 60.0)
        .positionConversionFactor(rotateCoversionFactor)
        ;        

    rotationConfig.absoluteEncoder
      .velocityConversionFactor(360 / 60.0)
      .positionConversionFactor(360)
      .inverted(true);

    rotationConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.8/60.0*2)
      .positionWrappingEnabled(false)
      .positionWrappingInputRange(0, 360)
      ;
    return rotationConfig;
  }
}
