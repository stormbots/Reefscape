// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeGrabber;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

/** Add your docs here. */
class AlgaeMech2d {
    double angledelta = 15;
    double barlength = 24;

    public Mechanism2d mech = new Mechanism2d(36, 72);
    MechanismRoot2d root = mech.getRoot("AlgaeGrabber", 6, 36);
    // MechanismLigament2d intakebarRelative =  root.append(new MechanismLigament2d("AlgaeIntakeBarRelative", 24, -90));
    MechanismLigament2d intakebar = root.append(new MechanismLigament2d("AlgaeIntakeBar", 24, -90));
    MechanismLigament2d shooterbar = root.append(new MechanismLigament2d("AlgaeShooterBar", 24, -90 + 10));
    MechanismLigament2d intake = intakebar.append(new MechanismLigament2d("AlgaeIntake", 0, 90));
    MechanismLigament2d shooter = shooterbar.append(new MechanismLigament2d("AlgaeShooter", 0, 90));

    MechanismLigament2d armPlant = root.append(new MechanismLigament2d("simBar", 30, 0));

    Supplier<Angle> getAngle;
    Supplier<AngularVelocity> getIntakeSpeed;
    Supplier<AngularVelocity> getShooterSpeed;
    Supplier<Angle> getSimAngle;

    public AlgaeMech2d(
        Supplier<Angle> angleSupplier,
        Supplier<AngularVelocity> intakeSpeedSupplier,
        Supplier<AngularVelocity> shooterSpeedSupplier,
        Supplier<Angle> angleSupplierSim
    ) {
      this.getAngle = angleSupplier;
      this.getIntakeSpeed = intakeSpeedSupplier;
      this.getShooterSpeed = shooterSpeedSupplier;
      this.getSimAngle = angleSupplier;

      var barweight = 10;
      // intakebarRelative.setColor(new Color8Bit(Color.kRed));
      // intakebarRelative.setLineWeight(barweight - 1);

      intakebar.setColor(new Color8Bit(Color.kGray));
      shooterbar.setColor(new Color8Bit(Color.kGray));
      intakebar.setLineWeight(barweight);
      shooterbar.setLineWeight(barweight);

      intake.setColor(new Color8Bit(Color.kDarkGreen));
      shooter.setColor(new Color8Bit(Color.kDarkRed));
      intake.setLineWeight(barweight);
      shooter.setLineWeight(barweight);
      armPlant.setLineWeight(1);

      if(Robot.isReal())armPlant.setLength(0);
      
      SmartDashboard.putData("mechanism/algaegrabber", mech);
    }

    //TODO: update Mechanism2d to take IO::Inputs once advantagekit is in
    public void update() {
      var angle = getAngle.get().in(Degree);
      intakebar.setAngle(new Rotation2d(Math.toRadians(angle)));
      shooterbar.setAngle(new Rotation2d(Math.toRadians(angle + angledelta)));

      intake.setLength(getIntakeSpeed.get().in(RPM) / 5760.0 * barlength / 4);
      shooter.setLength(getShooterSpeed.get().in(RPM) / 5760.0 * barlength / 4);

      // This is mostly to validate absolute vs relative, since they should be identical
    //   intakebarRelative.setAngle(new Rotation2d(Math.toRadians(armMotor.getEncoder().getPosition())));
    
      armPlant.setAngle(getSimAngle.get().in(Degree));
    }
  }

