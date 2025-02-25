// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

public class CoralIntakeMech2d{

    private final double coralIntakeLength = 10;

    private double maxIPS = 5760; //approximate 

    LoggedMechanism2d mech = new LoggedMechanism2d(36, 72);
    LoggedMechanismRoot2d root = mech.getRoot("CoralIntake", 6, 36);
    LoggedMechanismLigament2d intakePivot = root.append(new LoggedMechanismLigament2d("CoralIntakePivot", coralIntakeLength, 0));
    LoggedMechanismLigament2d intakeRollers = intakePivot.append(new LoggedMechanismLigament2d("CoralIntakeRollers", 0, 180));

    //Help visualize the simulated system, which is sometimes handy
    LoggedMechanismLigament2d armPlant = root.append(new LoggedMechanismLigament2d("CoralIntakeSim", coralIntakeLength+1, 0));

    public CoralIntakeMech2d() {
        double barweight = 10;

        intakePivot.setColor(new Color8Bit(Color.kGray));
        intakePivot.setLineWeight(barweight);

        armPlant.setLineWeight(1);

        if(Robot.isReal())armPlant.setLength(0);
        intakeRollers.setLineWeight(barweight+2);
        SmartDashboard.putData("mechanism/intake", mech);
        Logger.recordOutput("CoralIntake/mechanism", mech);

    }

    //TODO: update Mechanism2d to take IO::Inputs once advantagekit is in
    public void update(Angle pivotAngle, LinearVelocity shooterSpeed ) {
        armPlant.setLength(0);
        update(pivotAngle, shooterSpeed,Degrees.of(0));
    }

    public void update(Angle pivotAngle, LinearVelocity shooterSpeed  , Angle plantAngle) {
        intakePivot.setAngle(pivotAngle.in(Degrees));
        armPlant.setAngle(plantAngle.in(Degrees));

        double speed = shooterSpeed.in(InchesPerSecond);
        intakeRollers.setColor(speed>0 ? new Color8Bit(Color.kLime) : new Color8Bit(Color.kOrange));
        intakeRollers.setLength(shooterSpeed.in(InchesPerSecond) / maxIPS * coralIntakeLength / 4);
    }

}
