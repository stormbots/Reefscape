// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Leds;

import com.stormbots.BlinkenPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Leds extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  Servo blinkin1 = new Servo(7);
  Servo blinkin2 = new Servo(8);

  AddressableLEDSim sim = new AddressableLEDSim(ledStrip);

  /** Creates a new LEDs. */
  public Leds() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(8);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    blinkin1.setBoundsMicroseconds(2125, 1501, 1500, 1499, 1000);
    blinkin2.setBoundsMicroseconds(2125, 1501, 1500, 1499, 1000);

    //Run during the match
    setDefaultCommand(showTeamColor());
    schedulePattern(stormy().withTimeout(1));
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }
    
  public int matchBrightnessScaling(int disabledBrightness, int enabledBrightness) {
    if (DriverStation.isDisabled()){
      return disabledBrightness;
    }
    return enabledBrightness;
  }

  public void setColor(Color color) {
    int red = (int)(color.red*255);
    int green = (int)(color.green*255);
    int blue = (int)(color.blue*255);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  /**
   * @param color
   * @param brightness between 0 and 100
   */
  public void setColor(Color color, int brightness) {
    var hsv = new HSVColor(color);
    hsv.value = (int) (hsv.value*brightness/100.0);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hsv.hue, hsv.saturation, hsv.value);
    }
  }

  /** Only works for native LED strips; Blinkens need named color */
  private void setRGB(int red, int green, int blue) {
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public Command showTeamColor() {
    return run(()->{
      var color = DriverStation.getAlliance().orElse(Alliance.Blue);

      double timer = Timer.getMatchTime();

      //Endgame timer
      if (DriverStation.isTeleop() && timer>0 && timer<20) {
        this.setColor(Color.kSkyBlue);
        blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidSkyBlue.us());
        blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidSkyBlue.us());
        return;
      }

      if (color == DriverStation.Alliance.Red) {
        this.setColor(Color.kRed, this.matchBrightnessScaling(10, 100));
        blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidRed.us());
        blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidRed.us());
      }
      if (color == DriverStation.Alliance.Blue) {
        this.setColor(Color.kBlue, this.matchBrightnessScaling(10, 100));
        blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidBlue.us());
        blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidBlue.us());
      }
    })
    .finallyDo(this::setBlinkinTo5VStrip)
    ;
  }

  /** Fix the blinkins accidentally setting to 12V mode due to errant pulses. 
   * Should be used as a finallydo on default commands
  */
  private void setBlinkinTo5VStrip(){
    this.blinkin1.setPulseTimeMicroseconds(2125);
    this.blinkin2.setPulseTimeMicroseconds(2125);
  }

  public Command branch() {
    return run( ()->{
      setColor(Color.kPurple);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidViolet.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidViolet.us());
    });
  }

  public Command coral() {
    return run( ()->{
      setColor(Color.kWhite);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidWhite.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidWhite.us());
    });
  }

  public Command algae() {
    return run( ()->{
      setColor(Color.kTurquoise);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidLawnGreen.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidLawnGreen.us());
    });
  }

  public Command pink() {
    return run(
      ()->{this.setColor(Color.kHotPink);
      this.blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidHotPink.us());
      this.blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidHotPink.us());
    });
  }

  public Command stormy() {
    return run( ()->{
      LEDPattern.gradient(GradientType.kContinuous, 
        Color.kWhite,Color.kBlue,Color.kBlue,Color.kBlue,Color.kBlue,Color.kBlue
      )
      .offsetBy((int)(Timer.getFPGATimestamp()*5))
      .applyTo(ledBuffer)
      ;
    }).ignoringDisable(true);
  }


  /** Schedules a command to run. Intended to run decorated versions of the 
   * LED pattern without interrupting command sequences
   */
  public Command schedulePattern(Command ledCommand){
    //Only run LED patterns. >:(
    if(!ledCommand.hasRequirement(this)) return new InstantCommand();
    // Schedule it and run it until something else takes over
    return new InstantCommand(()->ledCommand.ignoringDisable(true)
    .ignoringDisable(true)
    .schedule());
  }

}
