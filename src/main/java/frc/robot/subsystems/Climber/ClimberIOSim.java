// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {  
  // private DCMotorSim sim = new DCMotorSim(
  //         LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0, 0),
  //         DCMotor.getNeoVortex(1));

  private SparkFlex sparkFlex = new SparkFlex(6, MotorType.kBrushless);
  private DCMotor dcmotor = DCMotor.getNeoVortex(1);
  private SparkFlexSim sparkSim = new SparkFlexSim(sparkFlex, dcmotor);

  private LinearSystem plant = LinearSystemId.createSingleJointedArmSystem(dcmotor, 4, 100);
  // private LinearSystem plant = LinearSystemId.createElevatorSystem(dcmotor, 10, 0.030, 10);
  private DCMotorSim sim = new DCMotorSim(plant, dcmotor);
  private double appliedVolts = 0.0;

  public ClimberIOSim(){
    //crudely attempt to define where we expect the simulation to start
    var startPosition=100; //degrees
    sim.setState(Math.toRadians(startPosition), 0);
    sparkSim.enable();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var dt = 0.02;
    var vbus = 12;
    inputs.climberVoltage = sparkSim.getAppliedOutput() * vbus;
    //output of the system is now known; We can finally update the simulation
    sim.setInputVoltage(inputs.climberVoltage);

    sim.update(dt);

    var rps = sim.getAngularVelocityRPM()*60;

    sparkSim.iterate(rps, vbus, dt);
    sparkSim.getAbsoluteEncoderSim().iterate(rps, dt);
    sparkSim.getRelativeEncoderSim().iterate(rps, dt);

    //Update the inputs for the log using the system state
    inputs.climberRelativeAngle = Units.rotationsToDegrees(sim.getAngularPositionRotations());
    inputs.climberVoltage = sim.getInputVoltage();
    inputs.climberCurrentDraw = sim.getCurrentDrawAmps();
    inputs.climberAbsoluteAngle = sim.getAngularPosition().in(Degrees);
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
}
