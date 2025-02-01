// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class ClimberVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d climber;
  private final String key;

  public ClimberVisualizer(String key) {
    this.key = key;
    mechanism =
        new LoggedMechanism2d(
            Units.inchesToMeters(24), Units.inchesToMeters(24), new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot(key, Units.inchesToMeters(12), Units.inchesToMeters(12)); // placeholder
    climber = new LoggedMechanismLigament2d("climber", Units.inchesToMeters(12), 0); // placeholder
    root.append(climber);
  }

  public void update(double climberAngleDegrees) {
    climber.setAngle(climberAngleDegrees);
    Logger.recordOutput("climber/Mechanism2d/" + key, mechanism);
  }
}
