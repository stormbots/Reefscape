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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  SparkFlex motor = new SparkFlex(9, MotorType.kBrushless);
  ClimberIOInputs inputs = new ClimberIOInputs();

  public ClimberIOReal() {
  }

  @Override
  public void updateInputs(ClimberIOInputs newinputs) {
    //disregard incoming inputs and just run it. 
    // inputs.climberAbsoluteAngle = motor.getAbsoluteEncoder().getPosition();
    // inputs.climberCurrentDraw = motor.getOutputCurrent();
    // inputs.climberRelativeAngle = motor.getEncoder().getPosition();
    // inputs.climberVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    // inputs.climberVelocity = motor.getAbsoluteEncoder().getVelocity();
  }

  @Override
  public double getVelocity() {
    return motor.getAbsoluteEncoder().getVelocity();
  }

  @Override
  public void setReference(double degrees) {
    motor.getClosedLoopController().setReference(degrees, ControlType.kPosition);
  }

  @Override
  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    motor.configure(config, resetMode, persistMode);
  }
  @Override
  public void configureAsync(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    motor.configureAsync(config, resetMode, persistMode);
  }

  @Override
  public void setIdleMode(IdleMode idleMode) {
    configureAsync(new SparkFlexConfig().idleMode(idleMode), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setRelativeEncoderPosition(double position){
    motor.getEncoder().setPosition(position);
  }

  public void stop(){
    motor.stopMotor();
  }

}
