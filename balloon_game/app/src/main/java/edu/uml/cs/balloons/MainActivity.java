package edu.uml.cs.balloons;

import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.drawable.Drawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.ros.android.RosActivity;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
public class MainActivity extends RosActivity {

    public final NodesStartedManager nodesStartedManager;
    public final TimeLeftSub timeLeftSub;
    public final GameStateSub gameStateSub;
    public final ScorePub scorePub;
    public final StatusBarPub statusBarPub;
    public final AppActivePub appActivePub;
    public final GameZoneDetectorSub gameZoneDetectorSub;
    public final URI rosMaster;
    private static MainActivity instance;

    public GameElements gameElements;
    public GameView gameView;

    public static MainActivity getInstance() {
        return instance;
    }

    protected MainActivity() {
        super("Balloons_Main" + GraphName.newAnonymous(),
                "Balloons_Main",
                getUri());
        instance = this;

        nodesStartedManager = new NodesStartedManager();
        timeLeftSub = new TimeLeftSub();
        gameStateSub = new GameStateSub();
        scorePub = new ScorePub();
        statusBarPub = new StatusBarPub();
        appActivePub = new AppActivePub();
        gameZoneDetectorSub = new GameZoneDetectorSub();
        rosMaster = getUri();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getPermission();

        gameElements = new GameElements(getApplicationContext());
        gameView = new GameView(getApplicationContext(), gameElements);

        TextView tv = (TextView)findViewById(R.id.ros_master_tv);
        tv.setText(getUri().toString());
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        try {
            java.net.Socket socket =
                    new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration =
                    NodeConfiguration.newPublic(
                            local_network_address.getHostAddress(),
                            getMasterUri());
            Log.d("Balloons", local_network_address.getHostAddress() + " " + getMasterUri());
            nodeMainExecutor.execute(timeLeftSub, nodeConfiguration);
            nodeMainExecutor.execute(gameStateSub, nodeConfiguration);
            nodeMainExecutor.execute(scorePub, nodeConfiguration);
            nodeMainExecutor.execute(statusBarPub, nodeConfiguration);
            nodeMainExecutor.execute(appActivePub, nodeConfiguration);
            nodeMainExecutor.execute(gameZoneDetectorSub, nodeConfiguration);
        } catch (IOException e) {
            // Socket problem
            Log.e(
                    "Balloons",
                    "socket error trying to get networking information from the master uri");
        }

        Log.d("wtf", "onclick registered");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        android.os.Process.killProcess(android.os.Process.myPid());
        Log.d("Balloons", "destroying");
    }

    public void enableGameButton() {
        final Button begin_button = (Button) findViewById(R.id.begin_button);
        runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    begin_button.setBackgroundResource(R.drawable.bg_button_ready);
                    begin_button.setText("BEGIN");
                }
            }
        );
        begin_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(instance, BalloonActivity.class);
                startActivity(intent);
            }
        });
    }

    public static URI getUri() {
        //Find the directory for the SD Card using the API
        //*Don't* hardcode "/sdcard"
        File sdcard = Environment.getExternalStorageDirectory();

        //Get the text file
        File file = new File(sdcard,"ros_master.txt");

        //Read text from file
        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            while ((line = br.readLine()) != null) {
                text.append(line);
            }
            br.close();
        }
        catch (IOException e) {
            Log.d(
                    "Balloons",
                    "ros_master.txt not found, reverting to default 'http://robot-lab1:11311'");
            Log.d("Balloons", e.toString());
            return URI.create("http://robot-lab1:11311");
            //You'll need to add proper error handling here
        }
        Log.d("Balloons", text.toString());
        return URI.create(text.toString());
    }

    private void getPermission() {
        // if we are newer than lollipop we need to request access at runtime
        if (Build.VERSION.SDK_INT > Build.VERSION_CODES.LOLLIPOP_MR1) {
            String[] perms = {"android.permission.READ_EXTERNAL_STORAGE"};
            int permsRequestCode = 200;
            requestPermissions(perms, permsRequestCode);
        }
    }

    @Override
    public void onRequestPermissionsResult(int permsRequestCode, String[] permissions, int[] grantResults){
        switch(permsRequestCode){
            case 200:
                boolean readAccepted = grantResults[0] == PackageManager.PERMISSION_GRANTED;
                Log.d("Balloons", "android.permission.READ_EXTERNAL_STORAGE runtime permission status: " + readAccepted);
                break;
        }
    }
}
