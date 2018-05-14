package edu.uml.cs.balloons;

import android.util.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import std_msgs.String;

/**
 * Created by csrobot on 12/28/16.
 */

public class GameStateSub implements NodeMain {
    private java.lang.String lastState;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("game_state_sub").join(GraphName.newAnonymous());
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Subscriber<std_msgs.String> subscriber =
                connectedNode.newSubscriber("/experiment/state", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                lastState = message.getData();
                if (MainActivity.getInstance().gameView != null) {
                    Log.d("Balloons", "I heard: \"" + message.getData() + "\"");
                    MainActivity.getInstance().gameView.setGameState(message.getData());
                } else {
                    Log.d("Balloons", "I heard: \"" + message.getData() + "\", but it was discarded.");
                }
            }
        });
        MainActivity.getInstance().nodesStartedManager.gameStateSubReady();
    }

    @Override
    public void onShutdown(Node node) {}

    @Override
    public void onShutdownComplete(Node node) {}

    @Override
    public void onError(Node node, Throwable throwable) {}

    public java.lang.String getLastState() {
        return lastState;
    }
}
