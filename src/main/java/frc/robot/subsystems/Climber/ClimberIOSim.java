// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {  

  private SparkFlex sparkFlex = new SparkFlex(9, MotorType.kBrushless);
  private DCMotor dcmotor = DCMotor.getNeoVortex(1);
  private SparkFlexSim sparkSim = new SparkFlexSim(sparkFlex, dcmotor);
  double startPosition=110; //degrees

  double momentArm=Inches.of(4).in(Meter);
  private SingleJointedArmSim sim = new SingleJointedArmSim(dcmotor, 
    10*10, 
    0.5*momentArm*momentArm,
    momentArm, 
    Math.toRadians(-600), 
    Math.toRadians(1100), 
    true, 
    Math.toRadians(startPosition)
  );  

  public ClimberIOSim(){
    sim.setState(Math.toRadians(startPosition), 0);
    sparkSim.setPosition(startPosition);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var dt = 0.02;
    var vbus = 12;
    //log the current system state
    inputs.climberVoltage = sparkSim.getAppliedOutput() * vbus;
    inputs.climberCurrentDraw = sparkSim.getMotorCurrent();
    inputs.climberRelativeAngle = getPosition();
    inputs.climberAbsoluteAngle = getPosition();

    //TODO: Battery sim
    // climberSim.setInput(sim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    // climberSim.update(0.02);
    // sim.iterate(Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.02);
    // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    sim.setInputVoltage(inputs.climberVoltage);
    sim.update(dt);
    var angVel = RadiansPerSecond.of(sim.getVelocityRadPerSec()).in(DegreesPerSecond);
    sparkSim.iterate(angVel, vbus, dt);
  }

  @Override
  public void setReference(double angle) {
    SmartDashboard.putNumber("sim/reference", angle);

    sparkFlex.getClosedLoopController().setReference(angle,ControlType.kPosition);
  }

  @Override
  public void setIdleMode(IdleMode idleMode) {
    var conf = new SparkFlexConfig().idleMode(idleMode);
    configureAsync(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public double getPosition() {
    return sparkSim.getAbsoluteEncoderSim().getPosition();
  }

  @Override
  public void configureAsync(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    sparkFlex.configure(config, resetMode, persistMode);
  }

  @Override
  public void configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    sparkFlex.configure(config, resetMode, persistMode);
  }

  // @Override
  public void setRelativeEncoderPosition(double position){
    sparkFlex.getEncoder().setPosition(position);
  }
}
