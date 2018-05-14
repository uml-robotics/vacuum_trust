package chuta.rosandroid;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

//import std_msgs.String;

/**
 * Created by chuta on 1/13/2017.
 */

public class StringTupleTalker extends AbstractNodeMain{
    private String topicName;
    private BlockingQueue<Boolean> queue;
    public StringTupleTalker(String name) {
        this.topicName = name;
    }
    public void enqueue(Boolean value)
    {
        queue.add(value);
    }
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosandroid/" + this.topicName); //note cannot conflict
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<std_msgs.Bool> publisher =
                connectedNode.newPublisher(topicName, std_msgs.Bool._TYPE);
        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceDiNumber;

            @Override
            protected void setup() {
                queue = new ArrayBlockingQueue<>(100);
            }

            @Override
            protected void loop() throws InterruptedException {
                std_msgs.Bool msg = publisher.newMessage();
                msg.setData(queue.take());
                publisher.publish(msg);
                //Thread.sleep(1000); //TODO change if necessary
            }
        });
    }
}
