package org.firstinspires.ftc.teamcode.DSYS;

public class Base {
    public static boolean intIsParsable(String input) {
        try {
            Integer.parseInt(input);
            return true;
        } catch (final NumberFormatException e) {
            return false;
        }
    }

    public static boolean floatIsParsable(String input) {
        try {
            Float.parseFloat(input);
            return true;
        } catch (final NumberFormatException e) {
            return false;
        }
    }

    public static float ClampBetween(float value, float max, float min)
    {
        return Math.min(Math.max(value, min), max);
    }

    // No different from the default print functions, just a simpler syntax
    public static void println(Object output) {
        System.out.println(output);
    }
    public static void print(Object output) { System.out.print(output); }

    // Change color of console text
    public static void changeColor(String color) {
        print(color);
    }
}
