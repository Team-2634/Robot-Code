package frc.robot;

import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KeyListener implements NativeKeyListener {


    public void nativeKeyPressed(NativeKeyEvent e) {
        String a = NativeKeyEvent.getKeyText(e.getKeyCode());
        switch (a) {
            case "N":
                System.out.println("Nahida");
                break;
            case "R":
                System.out.println("Robotics");
                break;
            case "2":
                System.out.println("2634");
                SmartDashboard.putString("Test", "2");
                break;
            default:
                break;
        }
    }










}