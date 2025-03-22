package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.util.Color;

public class HSVColor{
    // private Color rgb;
    public int hue;
    public int saturation;
    public int value;
    public HSVColor(int hue, int saturation,int value){
      this.hue = hue;
      this.saturation = saturation;
      this.value = value;
    }
    
    public HSVColor(Color color){
      double r = color.red;
      double g = color.green;
      double b = color.blue;
      double max = Math.max(r, Math.max(g, b)); // maximum of r, g, b 
      double min = Math.min(r, Math.min(g, b)); // minimum of r, g, b 
      double range = max - min; // diff of cmax and cmin. 
      double h = -1, s = -1; 
      if (max == min) 
        h = 0; 
      else if (max == r) 
        h = ((60 * ((g - b) / range) + 360) % 360)/2; 
      else if (max == g) 
        h = ((60 * ((b - r) / range) + 120) % 360)/2; 
      else if (max == b) 
        h = ((60 * ((r - g) / range) + 240) % 360)/2; 
      if (max == 0) 
        s = 0; 
      else
        s = (range / max) * 255; 
      double v = max * 255; 
      this.hue = (int)Math.round(h);
      this.saturation = (int)Math.round(s);
      this.value = (int)Math.round(v);
    }
  }
