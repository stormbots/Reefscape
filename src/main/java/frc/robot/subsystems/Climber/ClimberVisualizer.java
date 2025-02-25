// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ClimberVisualizer {
  private String key;

  double offsetBecauseWrongAngleReferenceChosen = 90;

  // Create the basic mechanism construction
  LoggedMechanism2d mechanism = new LoggedMechanism2d(24, 24);
  LoggedMechanismRoot2d root = mechanism.getRoot("ClimberRoot", 12, 10);
  LoggedMechanismLigament2d pivot = root.append(new LoggedMechanismLigament2d("Climber", 4, 0));
  LoggedMechanismLigament2d top = pivot.append(new LoggedMechanismLigament2d("ClimberHook", 4, 0));
  LoggedMechanismLigament2d bot = pivot.append(new LoggedMechanismLigament2d("ClimberBottomBar", 9, -90));
  LoggedMechanismLigament2d foot = bot.append(new LoggedMechanismLigament2d("Climberfoot", 3, 90));


  public ClimberVisualizer(String key) {
    this.key = key;
    SmartDashboard.putData("mechanism/climber",mechanism);
  }

  public void update(double climberAngleDegrees) {
    pivot.setAngle(climberAngleDegrees+offsetBecauseWrongAngleReferenceChosen);
    Logger.recordOutput("climber/Mechanism2d/" + key, mechanism);
  }
}
