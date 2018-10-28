package java_template;

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author daniele
 */
public class RobotSimulation {

    private Robot robot;

    public RobotSimulation() {
        try {
            // Import a new robot with "robot" as name
            robot = new Robot("robot");
        } catch (IOException ex) {
            Logger.getLogger(RobotSimulation.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void run() {
        robot.run();
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {

        // Start a new simulation
        RobotSimulation RS = new RobotSimulation();
        RS.run();
    }

}
