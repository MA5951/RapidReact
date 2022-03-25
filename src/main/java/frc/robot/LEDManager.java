// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class LEDManager {
    private final int LED_LENGTH = 1;

    private static LEDManager ledManager;
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private State currentState;
    private boolean isStopped;
    private double lastChange = 0;

    private int m_rainbowFirstPixelHue;

    private enum State {
        IDLE,
        FLASHING,
        RAINBOW
    }

    private LEDManager() {
        led = new AddressableLED(PortMap.leds);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(buffer.getLength());
        led.start();
        isStopped = false;
    }

    private void modifyRainbow() {
        // For every pixel
        for (int i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        led.setData(buffer);
    }

    private void modifyFlashing() {
        double current = Timer.getFPGATimestamp();
        if (current - lastChange >= 2) {
            if (isStopped) {
                isStopped = false;
                led.start();
            } else {
                led.stop();
            }
            lastChange = current;
        }
    }

    private void setSolidColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }

    public void setRed() {
        currentState = State.IDLE;
        setSolidColor(255, 0, 0);
    }

    public void setOrange() {
        currentState = State.IDLE;
        setSolidColor(255, 215, 0);
    }

    public void setGreen() {
        currentState = State.IDLE;
        setSolidColor(0, 255, 0);
    }

    public void setBlinkingPurple() {
        currentState = State.FLASHING;
        setSolidColor(128, 0, 128);
    }

    public void setRainbow() {
        currentState = State.RAINBOW;
    }

    public void periodic() {
        switch (currentState) {
            case FLASHING:
                modifyFlashing();
                break;
            case RAINBOW:
                modifyRainbow();
                break;
            default:
                break;
        }
    }

    public static LEDManager getInstance() {
        if (ledManager == null) {
            ledManager = new LEDManager();
        }
        return ledManager;
    }
}
