package edu.uml.cs.balloons;

import android.util.Log;

import org.ros.internal.message.RawMessage;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Int32;

/**
 * Created by csrobot on 1/6/17.
 */

public class ScorePub extends AbstractNodeMain {
    Publisher<Int32> scorePub;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("score_pub").join(GraphName.newAnonymous());
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        scorePub = connectedNode.newPublisher("/experiment/balloons/score", std_msgs.Int32._TYPE);
        MainActivity.getInstance().nodesStartedManager.scorePubReady();
    }

    public void publish(int score) {
        std_msgs.Int32 msg = scorePub.newMessage();
        msg.setData(score);
        scorePub.publish(msg);
    }
}
