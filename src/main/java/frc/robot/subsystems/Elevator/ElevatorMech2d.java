// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ElevatorMech2d {
    //visual constants
    double angledelta = 15;
    double barlength = 24;
    //physical constants
    double scorerLinkGearRatio = 1.5;
    double rotationArmLength = 16; //inches
    double rotationArmClockingOffsetAtVertical=0;


    public Mechanism2d mech = new Mechanism2d(36, 72);
    MechanismRoot2d root = mech.getRoot("ElevatorRoot", 18, 0);
    MechanismLigament2d elevator = root.append(new MechanismLigament2d("Elevator", 7, 90));
    //This only exists to re-orient the rotator to a horizontal reference so later setAngles make sense
    MechanismLigament2d rotatorMount = elevator.append(new MechanismLigament2d("RotatorBase", 0, -90));
    MechanismLigament2d rotator = rotatorMount.append(new MechanismLigament2d("Rotator", rotationArmLength, 0));
    // MechanismLigament2d coral = rotator.append(new MechanismLigament2d("ElevatorCoral", 1, 0));
    MechanismLigament2d translatorSpeed = rotator.append(new MechanismLigament2d("Translator", 0, 0));
    //These are just fixed rigid bars for visual reference
    MechanismLigament2d translatorBarFwd = rotator.append(new MechanismLigament2d("ATranslatorBarA", 6, 0));
    MechanismLigament2d translatorBarRev = rotator.append(new MechanismLigament2d("ATranslatorBarB", -6, 0));
    //Visualize the rotator's relative encoder
    // MechanismLigament2d rotatorRelative = rotatorMount.append(new MechanismLigament2d("RotatorRelative", 13, 0));

    public ElevatorMech2d() {
      var barweight = 10;

      elevator.setColor(new Color8Bit(Color.kDarkGray));
      elevator.setLineWeight(barweight * 2);

      rotator.setColor(new Color8Bit(Color.kGray));
      rotator.setLineWeight(barweight);

      translatorBarFwd.setColor(new Color8Bit(Color.kDarkGreen));
      translatorBarFwd.setLineWeight(barweight/2);
      translatorBarRev.setColor(new Color8Bit(Color.kDarkRed));
      translatorBarRev.setLineWeight(barweight/2);

      translatorSpeed.setColor(new Color8Bit(Color.kWhite));
      translatorSpeed.setLineWeight(barweight/2+4);

      // coral.setColor(new Color8Bit(Color.kWhite));
      // coral.setLineWeight(barweight * 4);
      // coral.setLength(0);

      // rotatorRelative.setColor(new Color8Bit(Color.kRed));
      // rotatorRelative.setLineWeight(barweight/2-1);

      SmartDashboard.putData("mechanism/elevator", mech);
    }

    public void update(
        double height,
        double angle,
        double translatespeed
        ) {
    //   var height = elevatorMotor.getEncoder().getPosition();
    //   var angle = rotationMotor.getAbsoluteEncoder().getPosition();
    //   var translatespeed = coralOutMotor.getEncoder().getVelocity();
    //   var isCoralLoaded = haveCoral.getAsBoolean();

      elevator.setLength(height);
      rotator.setAngle(angle);
      // coral.setLength(isCoralLoaded ? 12 : 0);

      var scorerAngle = -angle // this forms a parallel bar
        + (angle-90)*scorerLinkGearRatio //Rotate according to the gear ratio
        + rotationArmClockingOffsetAtVertical; //offset
      translatorBarFwd.setAngle(scorerAngle);
      translatorBarRev.setAngle(scorerAngle);

      //This is to just visualize the relative scorer speed
      translatorSpeed.setAngle(scorerAngle);
      translatorSpeed.setLength(translatespeed / 5760 * 6);
      translatorSpeed.setColor(translatespeed>0 ? new Color8Bit(Color.kLime) : new Color8Bit(Color.kOrange));

    }
  }
