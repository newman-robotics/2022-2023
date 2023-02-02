package org.firstinspires.ftc.teamcode.DSYS.GUI;
import java.awt.*;
import javax.swing.*;

/**
 * @author Declan J. Scott
 * */

// Creates different GUI windows
public class Window {
    public static void TextAlert(String title, String alert){
        JFrame window = new JFrame(title);
        window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        JLabel alertLabel = new JLabel(alert, SwingConstants.CENTER);
        alertLabel.setPreferredSize(new Dimension(300, 100));
        window.getContentPane().add(alertLabel, BorderLayout.CENTER);

        window.setLocationRelativeTo(null);
        window.pack();
        window.setVisible(true);
    }
}
