package org.firstinspires.ftc.teamcode.DSYS;
import java.util.Scanner;

/**
 * @author Declan J. Scott
 * */

public class Input {
    public static Scanner scanner = new Scanner(System.in);

    public static String getString(String prompt)
    {
        System.out.print(prompt);
        return scanner.nextLine();
    }

    public static int getInt(String prompt)
    {
        int result = 0;
        boolean gotInt = false;
        do {
            System.out.print(prompt);
            String input = scanner.nextLine();

            if(Base.intIsParsable(input))
            {
                result = Integer.parseInt(input);
                gotInt = true;
            }
        }
        while(!gotInt);
        return result;
    }

    public static float getFloat(String prompt)
    {
        float result = 0;
        boolean gotFloat = false;
        do {
            System.out.print(prompt);
            String input = scanner.nextLine();

            if(Base.floatIsParsable(input))
            {
                result = Float.parseFloat(input);
                gotFloat = true;
            }
        }
        while(!gotFloat);
        return result;
    }

    public static char getChar(String prompt)
    {
        System.out.print(prompt);
        return scanner.nextLine().charAt(0);
    }

    public static boolean getBool(String prompt, char t, char f)
    {
        do {
            char input = Character.toUpperCase(getChar(prompt));
            if(input == Character.toUpperCase(t))
            {
                return true;
            } else if (input == Character.toUpperCase(f))
            {
                return false;
            }
        }
        while(true);
    }
}