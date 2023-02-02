package org.firstinspires.ftc.teamcode.DSYS;
import java.util.Scanner;
import java.io.*;

/**
 * @author Declan J. Scott
 * */

public class IO {

    // Read file contents
    public static String read(String path)
    {
        StringBuilder data = new StringBuilder();
        try {
            File f = new File(path);
            Scanner reader = new Scanner(f);
            while (reader.hasNextLine()) {
                String line = reader.nextLine();
                data.append(line).append("\n");
            }
            reader.close();

            return data.toString();
        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        return null;
    }

    // Write file contents
    public static void write(String path, String contents) {
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(path);
        } catch (IOException e) {
            e.printStackTrace();
        }
        assert fileWriter != null;
        PrintWriter printWriter = new PrintWriter(fileWriter);
        printWriter.print(contents);
        printWriter.close();
    }

    // Append file contents
    public static void append(String path, String contents) {
        String newContent = read(path) + contents;
        write(path, newContent);
    }

    // Check if file exists
    public static boolean fileExists(String path) {
        File toCheck = new File(path);
        return toCheck.exists();
    }
}