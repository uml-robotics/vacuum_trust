package edu.uml.cs.balloons;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Bool;

/**
 * Created by csrobot on 1/6/17.
 */

public class AppActivePub extends AbstractNodeMain {
    public Publisher<Bool> appActivePub;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("app_active_pub").join(GraphName.newAnonymous());
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        appActivePub = connectedNode.newPublisher("/experiment/balloons/app_active", Bool._TYPE);
        MainActivity.getInstance().nodesStartedManager.appActivePubReady();
    }

    public void publish(boolean active) {
        Log.d("Balloons", "appactivepub " + active);
        Bool msg = appActivePub.newMessage();
        msg.setData(active);
        appActivePub.publish(msg);
    }
}
