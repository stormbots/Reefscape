// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class ElevatorVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;
  private final String key;

  public ElevatorVisualizer(String key) {
    this.key = key;
    mechanism =
        new LoggedMechanism2d(
            Units.inchesToMeters(30), Units.inchesToMeters(120), new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            key,
            Units.inchesToMeters(22),
            Units.inchesToMeters(6)); // These values are nowhere near correct
    elevator = new LoggedMechanismLigament2d("elevator", 0, 90);
    root.append(elevator);
  }

  public void update(double elevatorHeightMeters) {
    elevator.setLength(elevatorHeightMeters);
    Logger.recordOutput("elevator/Mechanism2d/" + key, mechanism);
  }
}
