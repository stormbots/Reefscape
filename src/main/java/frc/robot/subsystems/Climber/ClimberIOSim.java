// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;


import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {
  private final DCMotor motorModel;
  private final SparkFlex flex;
  private final SparkFlexSim sim;
  private final SingleJointedArmSim climberSim;

  // private double appliedVolts = 0.0;
  public ClimberIOSim() {
    motorModel = DCMotor.getNeoVortex(1);
    flex = new SparkFlex(9, MotorType.kBrushless);
    sim = new SparkFlexSim(flex, motorModel);
    climberSim = new SingleJointedArmSim(motorModel, 360.0, SingleJointedArmSim.estimateMOI(.5, 3), 0.5, 0.0, 2*(Math.PI), true, 0.0);
    
    var config = new SparkFlexConfig();
    config.encoder.positionConversionFactor(1);
    config.absoluteEncoder.positionConversionFactor(360);

    config.inverted(true);
    config
        .closedLoop
        .outputRange(-0.5, 0.5)
        .p(1 / 30.0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        // .positionWrappingInputRange(0, 360);
        .positionWrappingInputRange(-180, 180);
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(5).voltageCompensation(12.0);
    config.absoluteEncoder.inverted(false);
    flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
    
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.setInput(sim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    climberSim.update(0.02);
    sim.iterate(Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));
    inputs.climberAbsoluteAngle = Units.radiansToDegrees(climberSim.getAngleRads());
    inputs.climberRelativeAngle = Units.radiansToDegrees(climberSim.getAngleRads());
    inputs.climberCurrentDraw = climberSim.getCurrentDrawAmps();
    inputs.climberVoltage = sim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    
  }

  @Override
  public void setReference(double angle) {
    flex.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  // @Override
  // public double getPosition() {
    
  // }
}
