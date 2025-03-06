// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scorer;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

/** Add your docs here. */
public class ScorerMotorConfigs {
  public static SparkFlexConfig armConfig;
  public static SparkFlexConfig scorerConfig;

  public static SparkBaseConfig getScorerConfig(){
    SparkBaseConfig coralOutConfig = new SparkMaxConfig()
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kBrake)
      .inverted(true); //Positive is in, Negative is out

    var coralOutConversionFactor = 1; //3.371;
    coralOutConfig.encoder
      .velocityConversionFactor(coralOutConversionFactor)
      .positionConversionFactor(coralOutConversionFactor)
      ;

    coralOutConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .velocityFF(1/5760.0)
      .p(0)
      .p(0.1, ClosedLoopSlot.kSlot1);
    return coralOutConfig;
  }



}
