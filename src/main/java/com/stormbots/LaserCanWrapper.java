package com.stormbots;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeter;

import java.util.Optional;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LaserCanWrapper implements Sendable{
  public LaserCan laserCan;
  private Distance breakBeamThreshold = Distance.ofBaseUnits(99, Inch);
  private boolean online = false;
  
  public LaserCanWrapper(int canID){
    laserCan = new LaserCan(canID);

    //Do a default config for short range
    configureShortRange();
  }

  public LaserCanWrapper setThreshhold(Distance breakBeamThreshold){
    this.breakBeamThreshold = breakBeamThreshold;
    return this;
  }

  public LaserCanWrapper configureShortRange(){
    try {
      laserCan.setRangingMode(RangingMode.SHORT);
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
      online = true;
    } catch (ConfigurationFailedException e){
      online = false;
    }
    return this;
  }

  public LaserCanWrapper configureLongRange(){
    try {
      laserCan.setRangingMode(RangingMode.LONG);
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
      online = true;
    } catch (ConfigurationFailedException e){
      online = false;
    }
    return this;
  }

  /**
   * Get the distance to the object;
   * @return Optional containing the value, if object properly detected
   */
  public Optional<Distance> getDistanceOptional(){
    if(online==false) return Optional.empty();
    var reading = laserCan.getMeasurement();
    if (reading == null){
      return Optional.empty();
    }
    if (reading.status!=laserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
      return Optional.empty();
    }
    return Optional.of(Units.Millimeters.of(reading.distance_mm));
  }

  public Trigger isBreakBeamTripped = new Trigger( () -> {
    Optional<Distance> distance = getDistanceOptional();
    if(distance.isEmpty()) return false;
    return distance.get().lt(breakBeamThreshold);
  });

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Online", ()->online, null);
    builder.addDoubleProperty("Distance(in)", ()->getDistanceOptional().orElse(Inches.of(-1)).in(Inches), null);
    builder.addBooleanProperty("BreakBeam", isBreakBeamTripped, null);
    builder.addDoubleProperty("Distance(mm)", ()->getDistanceOptional().orElse(Millimeter.of(-1)).in(Millimeter), null);
  }


}
