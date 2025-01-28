// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  SparkFlex motor = new SparkFlex(6, MotorType.kBrushless);

  public ClimberIOReal() {
    var conf = new SparkFlexConfig();
    conf.absoluteEncoder.positionConversionFactor(360);
    conf.closedLoop
        .outputRange(-0.1, 0.1)
        .p(0.1 / 90.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    motor.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setReference(double degrees) {
    motor.getClosedLoopController().setReference(degrees, ControlType.kPosition);
  }

  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  /** Note: Inputs doesn't do anything *other* than get logged. */
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberAbsoluteAngle = motor.getAbsoluteEncoder().getPosition();
    inputs.climberCurrentDraw = motor.getOutputCurrent();
    inputs.climberRelativeAngle = motor.getEncoder().getPosition();
    inputs.climberVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public void applyConfig(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    motor.configure(config, resetMode, persistMode);
  }
}
