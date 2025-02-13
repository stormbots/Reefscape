// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

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
      .smartCurrentLimit(36)
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      ;
    elevatorConfig.softLimit
    .forwardSoftLimit(40)//???? This is not enables and wont be tuned until we have compbot
    .reverseSoftLimit(0)
    .reverseSoftLimitEnabled(true);
    ;
    var elevatorConversionfactor = (51 - 12)/22.11;
    elevatorConfig.encoder
        .positionConversionFactor(elevatorConversionfactor)
        .velocityConversionFactor(elevatorConversionfactor / 60.0)
        ;
    elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1);

    return elevatorConfig;
  }




  public static SparkBaseConfig getScorerConfig(){
    SparkBaseConfig coralOutConfig = new SparkMaxConfig()
      .smartCurrentLimit(16)
      .idleMode(IdleMode.kCoast)
      .inverted(false); //Positive is in, Negative is out

    var coralOutConversionFactor = 3.371;
    coralOutConfig.encoder
      .velocityConversionFactor(coralOutConversionFactor / 60.0)
      .positionConversionFactor(coralOutConversionFactor)
      ;

    coralOutConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .velocityFF(1/5760.0)
      .p(0);
    return coralOutConfig;
  }



  public static SparkBaseConfig getRotationConfig(){
    SparkBaseConfig rotationConfig = new SparkMaxConfig()
      .smartCurrentLimit(32)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      ;

    rotationConfig.softLimit
      .reverseSoftLimit(-180)
      .forwardSoftLimit(180)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false)
      ;

    double rotateCoversionFactor = 87.053/2.5333;
    rotationConfig.encoder
        .velocityConversionFactor(rotateCoversionFactor / 60.0)
        .positionConversionFactor(rotateCoversionFactor)
        ;        

    rotationConfig.absoluteEncoder
      .velocityConversionFactor(360 / 60.0)
      .positionConversionFactor(360)
      .inverted(false);

    rotationConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(0.8/60.0)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 360)
      ;
    return rotationConfig;
  }
}
