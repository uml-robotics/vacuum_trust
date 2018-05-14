package edu.uml.cs.balloons;

/**
 * Created by csrobot on 2/22/17.
 */

public class NodesStartedManager {
    boolean appActivePub;
    boolean gameStateSub;
    boolean gameZoneDetectorSub;
    boolean scorePub;
    boolean statusBarPub;
    boolean timeLeftSub;

    public void enableGameIfReady() {
        if (appActivePub && gameStateSub && gameZoneDetectorSub && scorePub && statusBarPub && timeLeftSub) {
            MainActivity.getInstance().enableGameButton();
        }
    }

    public void appActivePubReady() {
        appActivePub = true;
        enableGameIfReady();
    }
    public void gameStateSubReady() {
        gameStateSub = true;
        enableGameIfReady();
    }
    public void gameZoneDetectorSubReady() {
        gameZoneDetectorSub = true;
        enableGameIfReady();
    }
    public void scorePubReady() {
        scorePub = true;
        enableGameIfReady();
    }
    public void statusBarPubReady() {
        statusBarPub = true;
        enableGameIfReady();
    }
    public void timeLeftSubReady() {
        timeLeftSub = true;
        enableGameIfReady();
    }
}
