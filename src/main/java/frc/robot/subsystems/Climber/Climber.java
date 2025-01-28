// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber() {
    SmartDashboard.putBoolean("simulate", false);
    if (RobotBase.isReal()) {
      this.io = new ClimberIOReal();
      // }else if(SmartDashboard.getBoolean("simulate", false)){
      // this.io = new ClimberIOSim();
    } else {
      this.io = new ClimberIO() {}; // Create a blank stub for replay
    }

    rateLimit.reset(io.getPosition());
    SmartDashboard.putData("mechanism/climber", mech);

    setDefaultCommand(setAngle(0));
  }

  @Override
  public void periodic() {
    // Update our simulation mechanism

    // update our logger

    io.updateInputs(
        inputs); // this updates everything in Real, so we don't need to write to inputs at any
    // point

    // This method will be called once per scheduler run
    updateMechanism();
  }

  public Command setAngle(double angle) {
    return run(() -> io.setReference(angle));
  }

  SlewRateLimiter rateLimit = new SlewRateLimiter(30);

  public void setPosition(double angle) {
    io.setReference(angle);
  }

  public void setIdleMode(IdleMode idleMode) {
    var config = new SparkFlexConfig().idleMode(idleMode);
    io.applyConfig(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // public void applyConfig(SparkFlexConfig config,ResetMode resetMode,PersistMode persistMode){
  //   io.applyConfig(config, resetMode, persistMode);
  // }

  // Create the basic mechanism construction
  Mechanism2d mech = new Mechanism2d(20, 20);
  MechanismRoot2d root = mech.getRoot("ClimberRoot", 10, 10);
  MechanismLigament2d pivot = root.append(new MechanismLigament2d("ClimberPivot", 4, 0));
  MechanismLigament2d top = pivot.append(new MechanismLigament2d("ClimberPivotTop", 4, 90));
  MechanismLigament2d bot = pivot.append(new MechanismLigament2d("ClimberPivotBottom", 4, -90));

  private void updateMechanism() {
    pivot.setAngle(new Rotation2d(Math.toRadians(io.getPosition())));
  }
}
