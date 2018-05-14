package edu.uml.cs.balloons;

import android.util.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import std_msgs.Bool;

/**
 * Created by csrobot on 12/28/16.
 */

public class GameZoneDetectorSub implements NodeMain {
    private boolean lastDetection = true;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("game_zone_detector_sub").join(GraphName.newAnonymous());
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Subscriber<Bool> subscriber =
                connectedNode.newSubscriber("/experiment/game_zone_detector/enabled", Bool._TYPE);
        subscriber.addMessageListener(new MessageListener<Bool>() {
            @Override
            public void onNewMessage(Bool message) {
                Log.d("Balloons", "I heard: \"" + message.getData() + "\"");
                if (MainActivity.getInstance().gameView != null) {
                    MainActivity.getInstance().gameView.getGameElements().setGameZoneDetector(message.getData());
                }
            }
        });
        MainActivity.getInstance().nodesStartedManager.gameZoneDetectorSubReady();
    }

    @Override
    public void onShutdown(Node node) {}

    @Override
    public void onShutdownComplete(Node node) {}

    @Override
    public void onError(Node node, Throwable throwable) {}

    public boolean getLastDetection() { return lastDetection; }
}
