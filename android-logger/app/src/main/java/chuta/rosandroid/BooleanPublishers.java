package chuta.rosandroid;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import std_msgs.Bool;

/**
 * Created by chuta on 1/20/2017.
 */

public class BooleanPublishers extends AbstractNodeMain{
    private String name;
    private String[] commands;
    private BlockingQueue<Integer> queue;
    public BooleanPublishers(String name, String[] commands) {
        this.name = name;
        this.commands = commands;
    }
    public void enqueue(int value)
    {
        queue.add(value);
    }
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosandroid/" + this.name); //note cannot conflict
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final ArrayList<Publisher<Bool> > publishers = new ArrayList<>();

        for (String c : commands) {
            Publisher<Bool> p =
                    connectedNode.newPublisher("androidlogger/" + name + "/" + c, std_msgs.Bool._TYPE);
            publishers.add(p);
        }
        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void setup() {
                queue = new ArrayBlockingQueue<Integer>(20);
            }

            @Override
            protected void loop() throws InterruptedException {
                int id = queue.take();
                std_msgs.Bool msg = publishers.get(id).newMessage();
                msg.setData(true);
                publishers.get(id).publish(msg);
                //Thread.sleep(1000); //TODO change if necessary
            }
        });
    }
}
