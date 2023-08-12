/* Online Java Compiler and Editor */
public class ActionRunner {

    public static void ActionRunner() {
        try {
            Thread t1 = new Thread(new Action("drive"));
            Thread t2 = new Thread(new Action("moveArm"));
            Thread t3 = new Thread(new Action("placeThingy"));
            Thread t4 = new Thread(new Action("DriveBackwards"));

            t1.start();
            t2.start();

            t1.join();
            t2.join();

            t3.start();
            t3.join();

            t4.start();
            t4.join();

            System.out.println("Fin");

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

class Action implements Runnable {
    String functionName;
    public Action(String functionName) {
        this.functionName= functionName;
    }

    public void run() {
        switch(functionName) {
        case("drive"):
            drive(10);
            break;
        case("moveArm"):
            moveArm(5);
            break;
        case("placeThingy"):
            placeThingy(15);
            break;
        case("DriveBackwards"):
            DriveBackwards(10);
            break;
        }
    }

    public void drive(int delay) {
        ActionHelpers.action("Drive Forward");
        ActionHelpers.Delay(delay);
        ActionHelpers.action("Stop Driving");
    }

    public void moveArm(int delay) {
        ActionHelpers.action("Arm moving");
        ActionHelpers.Delay(delay);
        ActionHelpers.action("Arm Ready");
    }

    public void placeThingy(int delay) {
        ActionHelpers.action("Placing");
        ActionHelpers.Delay(delay);
        ActionHelpers.action("Thingy Placed");
    }

    public void DriveBackwards(int delay) {
        ActionHelpers.action("Drive Backward");
        ActionHelpers.Delay(delay);
        ActionHelpers.action("Stop Driving");
    }
}

class ActionHelpers {
    public static void action(String log) {
        System.out.println(log);
    }

    public static void Delay(int delay) {
        try {
            Thread.sleep(delay);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }
}