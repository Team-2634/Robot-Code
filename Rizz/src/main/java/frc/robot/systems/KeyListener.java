package frc.robot.systems;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KeyListener implements NativeKeyListener{

    public void nativeKeyPressed(NativeKeyEvent e) {
            String a = NativeKeyEvent.getKeyText(e.getKeyCode());
            switch (a) {
                case "R":
                    System.out.print("Robotics");
                    break;
                case "P":
                    SmartDashboard.putString("Pluh", "Pluh activated");
                    break;
                default:
                    break;
            } {



            }



    }
    
}
