package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum LEDState {
  RAINBOW,
  WHITE,
  RED,
  PURPLE,
  FLASHING_PURPLE,
  YELLOW,
  FLASHING_YELLOW,
  FLAMES,
  ORANGE,
  FLASHING_ORANGE
  }  

public class LED extends SubsystemBase {
  LEDState state = LEDState.RAINBOW;
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;

  private static final int[] REDS = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
  private static final int[] GREENS = { 0, 45, 90, 135, 180, 225, 270, 315, 0, 45, 90, 135, 180, 225, 270, 315, 0, 45, 90, 135 };
  private static final int[] BLUES = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  private static final Random RANDOM = new Random();

  public LED() {
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(64);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    //leds.start();

  }

  public Command setRainbow(){
    return runOnce(
      () -> {
        state = LEDState.RAINBOW;
      }
    );
  }

  public Command setOrange(){
    return runOnce(
      () -> {
        state = LEDState.RAINBOW;
      }
    );
  }

  public Command setWhite(){
    return runOnce(
      () -> {
        state = LEDState.WHITE;
      }
    );
  }

  public Command setRed(){
    return runOnce(
      () -> {
        state = LEDState.RED;
      }
    );
  }

  public Command setOrangeFlashing(){
    return runOnce(
      () -> {
        state = LEDState.FLASHING_ORANGE;
      }
    );
  }
  
  public Command setYellow(){
    return runOnce(
      () -> {
        state = LEDState.YELLOW;
      }
    );
  }

  public Command setYellowFlashing(){
    return runOnce(
      () -> {
        state = LEDState.FLASHING_YELLOW;
      }
    );
  }

  public Command setPurple(){
    return runOnce(
      () -> {
        state = LEDState.PURPLE;
      }
    );
  }

  public Command setPurpleFlashing(){
    return runOnce(
      () -> {
        state = LEDState.FLASHING_PURPLE;
      }
    );
  }

  public Command swapYellowPurple(){
    return runOnce(
      () -> {
        if(state == LEDState.PURPLE){
          state = LEDState.YELLOW;
        }
        else if(state == LEDState.YELLOW){
          state = LEDState.PURPLE;
        }
        else{
          state = LEDState.YELLOW;
        }
      }
    );
  }

  
  public void setFlashing(boolean flash){
    if(flash){
        if(state == LEDState.PURPLE){
            state = LEDState.FLASHING_PURPLE;
        }
        if(state == LEDState.YELLOW){
            state = LEDState.FLASHING_YELLOW;
        }
    }
    else{
        if(state == LEDState.FLASHING_PURPLE){
            state = LEDState.PURPLE;
        }
        if(state == LEDState.FLASHING_YELLOW){
            state = LEDState.YELLOW;
        }
    }
  }

  public Command setFlashingCommand(boolean flash){
    return run(
        () -> {
            setFlashing(flash);
        }
    );
  }

  @Override
  public void periodic() {
        /*for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
   
   m_led.setData(m_ledBuffer);*/
   if (state == LEDState.RAINBOW) {
    rainbow();
   } 
   else if (state == LEDState.WHITE) {
    white();
   }
   else if (state == LEDState.FLASHING_ORANGE) {
    orange_flash();
   }
   else if (state == LEDState.ORANGE) {
    orange();
   }
   else if (state == LEDState.RED) {
    red();
   }
   else if (state == LEDState.PURPLE) {
    purple();
   }
   else if (state == LEDState.FLASHING_PURPLE) {
    purple_flash();
   }
   else if (state == LEDState.YELLOW) {
    yellow();
   }
   else if (state == LEDState.FLASHING_YELLOW){
    yellow_flash();
   }
   else if (state == LEDState.FLAMES){
    flames();
   }
   m_led.setData(m_ledBuffer);
  }
  int index = 0;
  private void flames(){
    if(index % 50 == 0){
      for (int i = 0; i < 20; i++) {
        int flicker = RANDOM.nextInt(30) + 225;
        m_ledBuffer.setRGB(i, flicker * GREENS[i] / 255, flicker * REDS[i] / 255, flicker * BLUES[i] / 255);
      }
    }
    index++;
  }

  double m_rainbowFirstPixelHue = 0;
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, (int) hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void red() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);
    }

    private void purple() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 128, 0, 128);
      }
      m_led.setData(m_ledBuffer);
    }
    
    private int iterations = 0;

    private void purple_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 5) % 2 == 0) {
          m_ledBuffer.setRGB(i, 128, 0, 128);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }

     }
     iterations++;
    }

    private void yellow_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 5) % 2 == 0) {
          m_ledBuffer.setRGB(i, 255, 255, 0);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }

     }
     iterations++;
    }
      private void yellow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          m_ledBuffer.setRGB(i, 255, 255, 0);
       }
       
       m_led.setData(m_ledBuffer);
        }

      private void white() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          m_ledBuffer.setRGB(i, 255, 255, 255);
       }
       
       m_led.setData(m_ledBuffer);
        }

      private void orange() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          m_ledBuffer.setRGB(i, 255, 79, 0);
       }
       
       m_led.setData(m_ledBuffer);
        }

    private void orange_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 5) % 2 == 0) {
          m_ledBuffer.setRGB(i, 255, 79, 0);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }
     }
     iterations++;
    }
  }

