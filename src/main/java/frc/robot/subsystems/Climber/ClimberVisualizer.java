// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ClimberVisualizer {
  private String key;

  double offsetBecauseWrongAngleReferenceChosen = 90;

  // Create the basic mechanism construction
  Mechanism2d mechanism = new Mechanism2d(24, 24);
  MechanismRoot2d root = mechanism.getRoot("ClimberRoot", 12, 10);
  MechanismLigament2d pivot = root.append(new MechanismLigament2d("Climber", 4, 0));
  MechanismLigament2d top = pivot.append(new MechanismLigament2d("ClimberHook", 4, 0));
  MechanismLigament2d bot = pivot.append(new MechanismLigament2d("ClimberBottomBar", 9, -90));
  MechanismLigament2d foot = bot.append(new MechanismLigament2d("Climberfoot", 3, 90));


  public ClimberVisualizer(String key) {
    this.key = key;
    SmartDashboard.putData("mechanism/climber",mechanism);
  }

  public void update(double climberAngleDegrees) {
    pivot.setAngle(climberAngleDegrees+offsetBecauseWrongAngleReferenceChosen);
  }
}
