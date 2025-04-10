// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ElevatorSimulation {
  SparkFlex elevatorMotor;
  SparkFlex rotationMotor;

  SparkFlexSim simElevatorMotor;
  SparkFlexSim simRotationMotor;
  
  public ElevatorSimulation(
    SparkFlex elevatorMotor,
    SparkFlex rotationMotor
  ){
    this.elevatorMotor = elevatorMotor;
    this.rotationMotor = rotationMotor;
    simElevatorMotor = new SparkFlexSim(this.elevatorMotor, DCMotor.getNeoVortex(1));
    simRotationMotor = new SparkFlexSim(this.rotationMotor, DCMotor.getNeoVortex(1));
  
    var startPosition=Degrees.of(75);
    simArm.setState(startPosition.in(Radians), 0);
    simRotationMotor.setPosition(startPosition.in(Degrees));
  }
  

  ElevatorSim simElevator = new ElevatorSim(
    DCMotor.getNeoVortex(1),
    18, 
    1, 
    Inches.of(1).in(Meter), 
    Inches.of(0).in(Meter), 
    Inches.of(48).in(Meter), 
    true, 
    0
  );

  double armlength = Inches.of(16).in(Meters);
  double armMass = Pounds.of(7.6).in(Kilogram);
  SingleJointedArmSim simArm = new SingleJointedArmSim(
    DCMotor.getNeoVortex(1), 
    5*5*64/18.0, 
    armMass*armlength*armlength, 
    armlength, 
    Degrees.of(-180).in(Radians), 
    Degrees.of(270).in(Radians), 
    true, 
    Degrees.of(90).in(Radians)
  );

  

  public void update() {
    // TODO Auto-generated method stub
    var vbus = 12;
    var dt = 0.02;

    simElevator.setInputVoltage(simElevatorMotor.getAppliedOutput()*vbus);
    simElevator.update(dt);

    simArm.setInputVoltage(simRotationMotor.getAppliedOutput()*vbus);
    simArm.update(dt);

    SmartDashboard.putNumber("arm/plant/angle", Math.toDegrees(simArm.getAngleRads()));

    simElevatorMotor.iterate(
      MetersPerSecond.of(simElevator.getVelocityMetersPerSecond()).in(InchesPerSecond),
      vbus,
      dt
    );

    simRotationMotor.iterate(
      RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond),
      vbus,
      dt
    );

  }

  public Angle getAngle(){
    return Radians.of(simArm.getAngleRads());
  }

}
