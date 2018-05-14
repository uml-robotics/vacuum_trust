package edu.uml.cs.balloons;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Bool;
import std_msgs.Int32;

/**
 * Created by csrobot on 1/6/17.
 */

public class StatusBarPub extends AbstractNodeMain {
    public Publisher<Bool> statusBarPub;
    private boolean isStatusBarActive;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("status_bar_active_pub").join(GraphName.newAnonymous());
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        statusBarPub = connectedNode.newPublisher("/experiment/balloons/status_bar_active", Bool._TYPE);
        MainActivity.getInstance().nodesStartedManager.statusBarPubReady();
    }

    public void publish(boolean active) {
        Log.d("Balloons", "statusbaractivepub " + active);
        isStatusBarActive = active;
        Bool msg = statusBarPub.newMessage();
        msg.setData(active);
        statusBarPub.publish(msg);
    }

    public boolean isStatusBarActive() {
        return isStatusBarActive;
    }
}
