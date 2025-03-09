// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class AlgaeSim {
    SparkFlex armMotor;
    SparkFlex intakeMotor;
    SparkFlex shooterMotor;
    SparkFlexSim simArmMotor;
    SparkFlexSim simShooterMotor;
    SparkFlexSim simIntakeMotor;

    public AlgaeSim(SparkFlex armMotor, SparkFlex intakeMotor, SparkFlex shooterMotor){
        this.armMotor = armMotor;
        this.intakeMotor = intakeMotor;
        this.shooterMotor = shooterMotor;
        simArmMotor = new SparkFlexSim(armMotor, DCMotor.getNeoVortex(1));
        simShooterMotor = new SparkFlexSim(shooterMotor, DCMotor.getNeoVortex(1));
        simIntakeMotor = new SparkFlexSim(intakeMotor, DCMotor.getNeoVortex(1));
    }

  FlywheelSim simIntake = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), 0.00002016125, 1
    ),
    DCMotor.getNeoVortex(1)
  );
  FlywheelSim simShooter = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), 0.00002016125*3, 1
    ),
    DCMotor.getNeoVortex(1)
  );
  
  SingleJointedArmSim simArm = new SingleJointedArmSim(
    DCMotor.getNeoVortex(1), 
    20*90/30.362,
    0.2,
    0.5,
    Degrees.of(-110).in(Radians),
    Degrees.of(20).in(Radians),
    true,
    Degrees.of(0).in(Radians)
  );

  public void update() {
    var dt = 0.02;
    var vbus = 12;

    simArm.setInputVoltage(simArmMotor.getAppliedOutput()*vbus);
    simArm.update(dt);

    double velocity = RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond);
    simArmMotor.getAbsoluteEncoderSim().iterate(velocity, dt);
    simArmMotor.iterate(
      RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond),
      vbus, dt
    );

    simIntake.setInputVoltage(simIntakeMotor.getAppliedOutput()*vbus);
    simIntake.update(dt);
    simIntakeMotor.iterate(
      simIntake.getAngularVelocity().in(RPM),
      vbus, dt
    );

    simShooter.setInputVoltage(simShooterMotor.getAppliedOutput()*vbus);
    simShooter.update(dt);
    simShooterMotor.iterate(
      simShooter.getAngularVelocity().in(RPM),
      vbus, dt
    );
  }

  /** Access the state of the simulated plant */
  public Angle getAngle(){
    return Radian.of(simArm.getAngleRads());
  }
  public AngularVelocity getIntakeSpeed(){
    return simShooter.getAngularVelocity();
  }
  public AngularVelocity getShooterSpeed(){
    return simIntake.getAngularVelocity();
  }

}
