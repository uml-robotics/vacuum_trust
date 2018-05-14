package uml_robotics.robotnexus;

import android.graphics.Bitmap;
import android.util.Log;

import org.json.JSONArray;

import java.util.zip.Checksum;

/**
 * Robot object used by the model and controller
 */


public class Robot {
    private enum State {ok, safe, help, dangerous, off, NOT_SET} // a robot is in one of these states at all times
    private String name; // name of robot
    private Integer image = null; // image id of what robot looks like
    private boolean visible = true; // for dismissed robots
    private int proximity; // how close is this robot
    private String id; // hidden identifier for a bot
    private String model; // the make of a robot
    private State currState;
    private JSONArray progression = new JSONArray(); // most recent progression for this robot
    private long statusHashValue; //checksum value computed from most recent update string

    public Robot(int rssi, String id) {
        this.proximity = rssi;
        this.id = id;
        this.currState = State.NOT_SET;
    }

    public void setStatusHashValue(long statusHashValue) {
        this.statusHashValue = statusHashValue;
    }

    public long getStatusHashValue() {
        return statusHashValue;
    }

    public void setVisible(boolean visible) {
        this.visible = visible;
    }

    public boolean isVisible() {
        return visible;
    }

    public void setProgression(JSONArray progression) {
        this.progression = progression;
    }

    public JSONArray getProgression() {
        return progression;
    }

    public void setProximity(int proximity) {
        this.proximity = proximity;
    }

    public int getProximity() {
        return proximity;
    }

    public void setModel(String model) {
        this.model = model;
    }

    public String getModel() {
        return model;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setCurrState(String currState) {
        this.currState = State.valueOf(currState);
    }

    public String getCurrState() {
        return currState.toString();
    }

    public void setImage(Integer image) {
        this.image = image;
    }

    public Integer getImage() {
        return image;
    }

    public String getId() {
        return id;
    }

    // how to compare robots to each other
    @Override
    public boolean equals(Object robot) {
        Robot robotPrime = (Robot)robot;
        Robot robotSelf = this;


        //Log.i("Robot.equals", robotPrime.isVisible() + ": prime");
        //Log.i("Robot.equals", robotSelf.isVisible() + ": self");

        if (!(robotSelf.getId().equals(robotPrime.getId()))) {
            return false;
        }
        if (robotSelf.isVisible() != robotPrime.isVisible()) {
            return false;
        }
        //if (robotSelf.getProximity() != robotPrime.getProximity()) {
        //    return false;
        //}
        if (!(robotSelf.getCurrState().equals(robotPrime.getCurrState()))) {
            return false;
        }
        if (!(robotSelf.getProgression().equals(robotPrime.getProgression()))) {
            return false;
        }


        return true;
    }

    // returns contents of a complete robot
    @Override
    public Object clone() {
        Robot robot = new Robot(this.getProximity(), this.getId());
        robot.setName(this.getName());
        robot.setCurrState(this.getCurrState());
        robot.setModel(this.getModel());
        robot.setImage(this.getImage());
        robot.setProgression(this.getProgression());
        robot.setVisible(this.isVisible());
        robot.setStatusHashValue(this.getStatusHashValue());
        return robot;
    }

    @Override
    public String toString() {
        return (name == null? "Null" : name) /*+ " - " + model +
                "\nStatus: " + currState.toString() + "\nProximity: " +
                proximity + "\nID: " + id*/;
     }
}
