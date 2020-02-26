/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Robot;
import frc.robot.Constants.MiscConstants;

/**
 * Add your docs here.
 */
public class StatusLED {

    private final AddressableLED m_statusLED = new AddressableLED(MiscConstants.kStatusLEDPort);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(MiscConstants.kStatusLEDLength);

    public StatusLED() {
        m_statusLED.setLength(m_ledBuffer.getLength());
        m_statusLED.setData(m_ledBuffer);
    }

    public void startLED() {
        m_statusLED.start();
    }

    public void setLEDColorValue(int ledNumber, int r, int g, int b) {
        for (var i = 0; i < ledNumber; i++) {
            m_ledBuffer.setRGB(ledNumber, r, g, b);
        }
        m_statusLED.setData(m_ledBuffer);
    }

    public void setLEDColor(String color, int ledNumber) {
        Robot.ledColor = color;
        switch (color) {
        default:
            setLEDColorValue(MiscConstants.kStatusLEDLength, 0, 255, 0);
            m_statusLED.setData(m_ledBuffer);

        case "blue":
            setLEDColorValue(MiscConstants.kStatusLEDLength, 0, 0, 255);
            m_statusLED.setData(m_ledBuffer);
            break;

        case "red":
            setLEDColorValue(MiscConstants.kStatusLEDLength, 255, 0, 0);
            m_statusLED.setData(m_ledBuffer);
            break;

        case "purple":
            setLEDColorValue(MiscConstants.kStatusLEDLength, 204, 0, 204);
            m_statusLED.setData(m_ledBuffer);
            break;

        case "orange":
            setLEDColorValue(MiscConstants.kStatusLEDLength, 255, 153, 51);
            m_statusLED.setData(m_ledBuffer);
            break;

        case "yellow":
            setLEDColorValue(MiscConstants.kStatusLEDLength, 255, 255, 0);
            m_statusLED.setData(m_ledBuffer);
            break;

        case "black":
            setLEDColorValue(MiscConstants.kStatusLEDLength, 0, 0, 0);
            m_statusLED.setData(m_ledBuffer);
            break;

        }
    }
}
