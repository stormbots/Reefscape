// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Leds;

import java.util.function.BooleanSupplier;

import com.stormbots.BlinkenPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Leds extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  Servo blinkin1 = new Servo(1);
  Servo blinkin2 = new Servo(2);


  /** Creates a new LEDs. */
  public Leds() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(16);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    blinkin1.setBoundsMicroseconds(2125, 1501, 1500, 1499, 1000);   
    blinkin2.setBoundsMicroseconds(2125, 1501, 1500, 1499, 1000);   
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
    // This method will be called once per scheduler run
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
    return new RunCommand(()->{
      var color = DriverStation.getAlliance().orElse(Alliance.Blue);

      double timer = Timer.getMatchTime();

      //Endgame timer
      if (DriverStation.isTeleop() && timer>0 && timer<20) {
        this.setColor(Color.kPurple);
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
    },this)
    .ignoringDisable(true)
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

  public Command branchInRange() {
    return run( ()->{
      setColor(Color.kPurple);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidViolet.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidViolet.us());
    });
  }

  public Command coralLoaded() {
    return run( ()->{
      setColor(Color.kWhite);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidWhite.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidWhite.us());
    });
  }

  public Command algaeLoaded() {
    return run( ()->{
      setColor(Color.kTurquoise);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidLawnGreen.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidLawnGreen.us());
    });
  }


}
