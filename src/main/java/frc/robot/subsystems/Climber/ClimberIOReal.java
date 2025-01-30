// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  SparkMax climbMotor = new SparkMax(6, MotorType.kBrushless);

  public ClimberIOReal() {
    var config = new SparkMaxConfig();
    config.encoder.positionConversionFactor((360 / (9424 / 203.0)));
    config.absoluteEncoder.positionConversionFactor(360);
    config.inverted(false);
    config
        .closedLoop
        .outputRange(-0.1, 0.1)
        .p(0.1 / 90.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true);
    // config.idleMode(IdleMode.kBrake).smartCurrentLimit(0).voltageCompensation(12.0);
    config.absoluteEncoder.inverted(true);

    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberAbsoluteAngle = climbMotor.getAbsoluteEncoder().getPosition();
    inputs.climberRelativeAngle = climbMotor.getEncoder().getPosition();
    inputs.climberVoltage = climbMotor.getAppliedOutput() * climbMotor.getBusVoltage();
    inputs.climberCurrentDraw = climbMotor.getOutputCurrent();
  }

  // @Override
  // public void setBrakeMode(boolean brakeMode) {
  //   if (brakeMode) {
  //     config.idleMode(IdleMode.kBrake);
  //   } else {
  //     config.idleMode(IdleMode.kCoast);
  //   }
  // }

  @Override
  public double getPosition() {
    return climbMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void setReference(double degrees) {
    climbMotor.getClosedLoopController().setReference(degrees, ControlType.kPosition);
  }

  @Override
  public void applyConfig(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    climbMotor.configure(config, resetMode, persistMode);
  }
}
