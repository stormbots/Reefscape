// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

/** Add your docs here. */
public class CoralIntakeSimulation {
    SparkFlex pivotMotor;
    SparkFlex intakeMotor;
    SparkFlexSim simPivotMotor;
    SparkFlexSim simRollerMotor;

    double startingAngle = 89;

    CoralIntakeSimulation(SparkFlex pivotMotor, SparkFlex intakeMotor){
        this.pivotMotor = pivotMotor;
        this.intakeMotor = intakeMotor;
        simPivotMotor = new SparkFlexSim(pivotMotor, DCMotor.getNeoVortex(1));
        simRollerMotor = new SparkFlexSim(intakeMotor, DCMotor.getNeoVortex(1));

        simPivot.setState(Degrees.of(startingAngle).in(Radians), 0);
        simPivotMotor.setPosition(startingAngle);
    }

    double momentArm=Inches.of(7).in(Meter);
    SingleJointedArmSim simPivot = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1), 
        40,
        SingleJointedArmSim.estimateMOI(momentArm, 3),
        momentArm,
        Degrees.of(-30).in(Radians),
        Degrees.of(95).in(Radians),
        true,
        Degrees.of(startingAngle).in(Radians)
    );

    FlywheelSim simRoller = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNeoVortex(1), 0.00002016125, 1
        ),
        DCMotor.getNeoVortex(1)
    );

    public void update() {
        var dt = 0.02;
        var vbus = RoboRioSim.getVInVoltage();


        simPivot.setInputVoltage(simPivotMotor.getAppliedOutput()*vbus);
        simPivot.update(dt);
        simPivotMotor.iterate(
            RadiansPerSecond.of(simPivot.getVelocityRadPerSec()).in(DegreesPerSecond),
            vbus, dt
        );
        simPivotMotor.getAbsoluteEncoderSim().iterate(RadiansPerSecond.of(simPivot.getVelocityRadPerSec()).in(DegreesPerSecond), dt);

        simRoller.setInputVoltage(simRollerMotor.getAppliedOutput()*vbus);
        simRoller.update(dt);
        simRollerMotor.iterate(
            simRoller.getAngularVelocity().in(RPM),
            vbus, dt
        );

        var amps = simPivot.getCurrentDrawAmps()+simRoller.getCurrentDrawAmps();
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(amps));
    }

    public Angle getSimAngle(){
        return Radians.of(simPivot.getAngleRads());
    }
}
