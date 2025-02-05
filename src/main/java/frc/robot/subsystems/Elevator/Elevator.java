// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public static double kScoringOffsetHeight = Units.inchesToMeters(3);

  private final String elevatorName;
  private final String armName;
  private final ElevatorIO elevatorIO;
  private final ArmIO armIO;

  private final ElevatorVisualizer bruhVisualizer;

  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  /** Creates a new Elevator. */
  public Elevator(String elevatorName, ElevatorIO elevatorIO, String armName, ArmIO armIO) {
    this.elevatorName = elevatorName;
    this.elevatorIO = elevatorIO;
    this.armName = armName;
    this.armIO = armIO;

    bruhVisualizer = new ElevatorVisualizer("bruh");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs(elevatorName, elevatorInputs);

    armIO.updateInputs(armInputs);
    Logger.processInputs(armName, armInputs);

    bruhVisualizer.update(elevatorInputs.heightMeters);
  }

  public Command runElevator(double volts) {
    return startEnd(() -> elevatorIO.setVoltage(volts), () -> elevatorIO.stop());
  }

  public void setHeightMeters(double meters) {
    PIDController pid = new PIDController(6, 0, 0);
    double error = meters - elevatorInputs.heightMeters;
    Logger.recordOutput("elevator/error", error);

    elevatorIO.setVoltage(-pid.calculate(error));
    Logger.recordOutput("elevator/pidOutput", pid.calculate(error));
  }

  public Command setElevatorHeightCommand(double meters) {
    return new RunCommand(() -> setHeightMeters(meters), this)
    // .until doesn't work, something about offset of the mechanism but not sure, will figure out
    // later.
    // .until(
    //     () -> {
    //       return Math.abs(inputs.heightMeters - meters) < 0.01
    //           && Math.abs(inputs.velocityMPS) < 0.03;
    //     });
    ;
  }

  public double getHeight(){
    return elevatorInputs.heightMeters;
  }

  public void setAngleRadians(double radians) {
    PIDController pid = new PIDController(10, 0, 0);
    double error = radians - armInputs.armRelativePositionRads;
    Logger.recordOutput("Elevator/errorDeg", Math.toDegrees(error));

    armIO.setVoltage(-pid.calculate(error));
    Logger.recordOutput("Elevator/pidOutput", pid.calculate(error));
  }

  public void setArmVoltage(double volts){
    armIO.setVoltage(volts);
  }

  public double getArmAngleRadians(){
    return armInputs.armRelativePositionRads;
  }

  public double getCoralScorerAngleRadians(){
    return armInputs.coralScorerPositionRads;
  }

  public Command setAngleRadiansCommand(double radians){
    return new RunCommand(()->setAngleRadians(radians), this);
  }

  public Translation2d getArmCoordinates(){
    double armLength = 0.41;
    double x = Math.cos(getArmAngleRadians())*armLength;
    double y = getHeight();
    y += Math.sin(getArmAngleRadians())*armLength;
    Logger.recordOutput("bruh", new Translation2d(x, y));
    return new Translation2d(x, y);

  }
}
