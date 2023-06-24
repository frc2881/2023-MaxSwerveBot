// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;

public final class Utils {

    public static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0.0;
        }
        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }

    /** Converts volts to PSI per the REV Analog Pressure Sensor datasheet. */
    public static double voltsToPsi(double sensorVoltage, double supplyVoltage) {
        return 250 * (sensorVoltage / supplyVoltage) - 25;
    }

    public static boolean isValueBetween(double value, double minValue, double maxValue) {
        return value >= minValue && value <= maxValue;
    }

    public static boolean isPoseInBounds(Pose2d value, Pose2d minPose, Pose2d maxPose) {
        return 
            isValueBetween(value.getX(), minPose.getX(), maxPose.getX()) &&
            isValueBetween(value.getY(), minPose.getY(), maxPose.getY());
    }

    private static final ObjectMapper objectMapper = new ObjectMapper();
    public static String objectToJson(Object o) {
      try {
        return objectMapper.writeValueAsString(o);
      } catch (Exception ex) {
        return "{}";
      }
    }

}
