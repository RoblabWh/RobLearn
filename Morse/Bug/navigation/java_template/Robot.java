/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package java_template;

import java.io.IOException;
import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import sensorsActuators.*;

/**
 *
 * @author daniele
 */
public class Robot {
    public SpeedActuator speedAct;
    private final IrSensor irSens;
    private final PoseSensor poseSens;
    private final DistanceSensor distSens;

    private final String name;

    public Robot(String _name) throws IOException {
        name = _name;

        speedAct = new SpeedActuator(name, "motion", "127.0.0.1", 4000);

        irSens = new IrSensor(name, "IR1", "127.0.0.1",4000);
        irSens.setSensorListener(new IrSensorListener());

        poseSens = new PoseSensor(name, "pose", "127.0.0.1",4000);
        poseSens.setSensorListener(new PoseSensorListener());

        distSens = new DistanceSensor(name, "prox", "127.0.0.1",4000);
        distSens.setSensorListener(new DistanceSensorListener());

    }

    class PoseSensorListener implements SensorListener {

        @Override
        public void onSense(int meas) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void onSense(double meas) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void onSense(double[] meas) {
//            for (double i : meas) {
//                System.out.println(i);
//            }
        }

        @Override
        public void onSense(String s, HashMap<String, Double> map) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

    }

    class IrSensorListener implements SensorListener {

        @Override
        public void onSense(int meas) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void onSense(double meas) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void onSense(double[] meas) {
//            for (double i : meas) {
//                System.out.println(i);
//            }
        }

        @Override
        public void onSense(String s, HashMap<String, Double> map) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

    }

    class DistanceSensorListener implements SensorListener {

        @Override
        public void onSense(int meas) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void onSense(double meas) {
            // System.out.println(meas);
        }

        @Override
        public void onSense(double[] meas) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public void onSense(String s, HashMap<String, Double> map) {
//            System.out.println(s);
//
//            Set o = map.keySet();
//            for ( Object a : o) {
//                System.out.println(a);
//                System.out.println(map.get(a));
//            }
        }

    }

    public void run() {
        System.out.println("Robot started");

        try {
            poseSens.sense();
            irSens.sense();
            distSens.sense();

            HashMap<String, Double> map = new HashMap<>();
            map.put("linVel", 1.);
            map.put("angVel", 1.);
            speedAct.act(map);

        } catch (IOException ex) {
            Logger.getLogger(Robot.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
