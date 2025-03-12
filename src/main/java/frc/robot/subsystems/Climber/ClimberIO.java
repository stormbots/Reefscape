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
  
  // @AutoLog
  public static class ClimberIOInputs {
    /** Degrees */
    double climberAbsoluteAngle = 0.0;
    /** Degrees */
    double climberRelativeAngle = 0.0;
    /** Volts */
    double climberVoltage = 0.0;
    /** Amps */
    double climberCurrentDraw = 0.0;
    /** Degrees/s */
    double climberVelocity = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setIdleMode(IdleMode idleMode) {}

  public default void setReference(double degrees) {}

  public default double getPosition() {
    return 0;
  }

  public default void configureAsync(SparkBaseConfig config, ResetMode resetMode,PersistMode persistMode){}
  public default void configure(SparkBaseConfig config, ResetMode resetMode,PersistMode persistMode){}

  public default void setRelativeEncoderPosition(double position){}
  public default double getVelocity() { return 0;}

  public default void stop(){}
}
