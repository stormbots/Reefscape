// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public interface ClimberIO {
  
  @AutoLog
  public static class ClimberIOInputs {
    double climberAbsoluteAngle = 0.0;
    double climberRelativeAngle = 0.0;
    double climberVoltage = 0.0;
    double climberCurrentDraw = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setIdleMode(IdleMode idleMode) {}

  public default void setReference(double degrees) {}

  public default double getPosition() {
    return 0;
  }

  public default void configureAsync(SparkBaseConfig config, ResetMode resetMode,PersistMode persistMode){}
  public default void configure(SparkBaseConfig config, ResetMode resetMode,PersistMode persistMode){}
}
