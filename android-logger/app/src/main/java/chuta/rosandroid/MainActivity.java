package chuta.rosandroid;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.graphics.Color;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.widget.DrawerLayout;
import android.support.v7.app.ActionBarDrawerToggle;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.support.design.widget.NavigationView;


import org.ros.android.RosActivity;

import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;


public class MainActivity extends RosActivity {

    private ArrayList<ArrayList<BooleanTalker> > talkerCollection;
    private final String TAG = "RosTester";
    private final String[] robotNames = {"Dirtdog", "Bender", "Roomba", "Neato", "Discovery", "Participant"}; //graphname cannot have numbers?
    //private final String[] robotNames = {"Dirtdog", "Bender"};
    private final String[] commands =
            {"Robot/Looking", "Robot/Touched", "Robot/Pickedup", "Robot/OnCharger", //0-3
             "Dirtbin/Removed", "Dirtbin/Emptied", "Dirtbin/Replaced", //4-6
             "Gamezone/Entered", "Gamezone/Left", //7-8
             "Button/Power", "Button/Spot", "Button/Clean", "Button/Max", "Button/Dock", "Button/Menu", "Button/Other", //9-15
             "Report/Phonelink", "Report/Gamezone", "Report/Robot", //16-18
             "Participant/RequestPause", "Participant/AskedQuestion", "Participant/SatDown", "Participant/StoodUp" //19 ~ 22
    };
    private ArrayList<BooleanPublishers> pubs;
    private ArrayAdapter<String> adapter;
    private ArrayList<String> logList;
    private File logFile;
    private int selectedRobot;

    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("ros android", "ros android");
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //talkerCollection = new ArrayList<>();
        //list will contain a list of all commands for each robot
        /*for (String name : robotNames)
        {
            talkerCollection.add(generateAllPublishers(name));
        }*/

        pubs = new ArrayList<>();
        for (String n : robotNames)
        {
            BooleanPublishers bp = new BooleanPublishers(n, commands);
            pubs.add(bp);
        }
        setContentView(R.layout.activity_main);
        NavigationView navigationView = (NavigationView) findViewById(R.id.nav_view);
        DrawerLayout dlayout = (DrawerLayout)findViewById(R.id.drawer_layout);
        dlayout.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(TAG, "Drawer Layout selected");
            }
        });


        ListView lv = (ListView)findViewById(R.id.left_drawer);
        logList = new ArrayList<>();
        adapter = new ArrayAdapter<String>(this, R.layout.adapter_layout, logList);
        lv.setAdapter(adapter);
        lv.setOnItemClickListener(new AdapterView.OnItemClickListener()
        {
            @Override
            public void onItemClick(AdapterView<?> adap, View v, int position,
                                    long arg3)
            {
                String value = (String)adap.getItemAtPosition(position);
                logList.remove(position);
                adapter.notifyDataSetChanged();
            }
        });
        File path = getExternalFilesDir(null);
        path.mkdirs();
        logFile = new File(path, getTimeStamp()+ ".txt");
        Log.d(TAG, path.getAbsolutePath());
        File[] files = path.listFiles();
        for (File f: files)
        {
            Log.d(TAG, "Found: " + f.getAbsolutePath());
        }
    }

    private String getTimeStamp()
    {
        SimpleDateFormat sdfDate = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss");//dd/MM/yyyy
        Date now = new Date();
        String strDate = sdfDate.format(now);
        return strDate;
    }

    private ArrayList<BooleanTalker> generateAllPublishers(String robotName) {
        ArrayList<BooleanTalker> btalkers = new ArrayList<>();
        for (String command : commands) {
            btalkers.add(new BooleanTalker(robotName + "/" + command));
        }
        return btalkers;
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
/*
        for (ArrayList<BooleanTalker> talkers : talkerCollection)
        {
            for (BooleanTalker ts : talkers)
            {
                nodeMainExecutor.execute(ts, nodeConfiguration);
            }
        }
*/
        for (BooleanPublishers bp : pubs)
        {
            nodeMainExecutor.execute(bp, nodeConfiguration);
        }

    }


    public void handleSendButton(View view)
    {
        /*
        if (view == findViewById(R.id.buttonRobot1)){
            talkerCollection.get(selectedRobot).get(0).enqueue(true);
        } else if (view == findViewById(R.id.buttonRobot2)){
            talkerCollection.get(selectedRobot).get(1).enqueue(true);
        } else if (view == findViewById(R.id.buttonRobot3)){
            talkerCollection.get(selectedRobot).get(2).enqueue(true);
        } else if (view == findViewById(R.id.buttonRobot4)){
            talkerCollection.get(selectedRobot).get(3).enqueue(true);
        } else if (view == findViewById(R.id.buttonDirtbin1)){
            talkerCollection.get(selectedRobot).get(4).enqueue(true);
        } else if (view == findViewById(R.id.buttonDirtbin2)){
            talkerCollection.get(selectedRobot).get(5).enqueue(true);
        } else if (view == findViewById(R.id.buttonDirtbin3)){
            talkerCollection.get(selectedRobot).get(6).enqueue(true);
        } else if (view == findViewById(R.id.buttonGamezone1)) {
            talkerCollection.get(selectedRobot).get(7).enqueue(true);
        } else if (view == findViewById(R.id.buttonGamezone2)) {
            talkerCollection.get(selectedRobot).get(8).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton1)) {
            talkerCollection.get(selectedRobot).get(9).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton2)) {
            talkerCollection.get(selectedRobot).get(10).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton3)) {
            talkerCollection.get(selectedRobot).get(11).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton4)) {
            talkerCollection.get(selectedRobot).get(12).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton5)) {
            talkerCollection.get(selectedRobot).get(13).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton6)) {
            talkerCollection.get(selectedRobot).get(14).enqueue(true);
        } else if (view == findViewById(R.id.buttonButton7)) {
        talkerCollection.get(selectedRobot).get(15).enqueue(true);
        } else if (view == findViewById(R.id.buttonReport1)) {
        talkerCollection.get(selectedRobot).get(16).enqueue(true);
        } else if (view == findViewById(R.id.buttonReport2)) {
            talkerCollection.get(selectedRobot).get(17).enqueue(true);
        } else if (view == findViewById(R.id.buttonReport3)) {
            talkerCollection.get(selectedRobot).get(18).enqueue(true);
        } else {
            Log.d(TAG, "Unknown button");
        }*/
        try {
            FileWriter fw = new FileWriter(logFile, true); //second arg true -> append mode
            int commandID;
            if (view == findViewById(R.id.buttonRobot1)) {
                commandID = 0;
            } else if (view == findViewById(R.id.buttonRobot2)) {
                commandID = 1;
            } else if (view == findViewById(R.id.buttonRobot3)) {
                commandID = 2;
            } else if (view == findViewById(R.id.buttonRobot4)) {
                commandID = 3;
            } else if (view == findViewById(R.id.buttonDirtbin1)) {
                commandID = 4;
            } else if (view == findViewById(R.id.buttonDirtbin2)) {
                commandID = 5;
            } else if (view == findViewById(R.id.buttonDirtbin3)) {
                commandID = 6;
            } else if (view == findViewById(R.id.buttonGamezone1)) {
                commandID = 7;
            } else if (view == findViewById(R.id.buttonGamezone2)) {
                commandID = 8;
            } else if (view == findViewById(R.id.buttonButton1)) {
                commandID = 9;
            } else if (view == findViewById(R.id.buttonButton2)) {
                commandID = 10;
            } else if (view == findViewById(R.id.buttonButton3)) {
                commandID = 11;
            } else if (view == findViewById(R.id.buttonButton4)) {
                commandID = 12;
            } else if (view == findViewById(R.id.buttonButton5)) {
                commandID = 13;
            } else if (view == findViewById(R.id.buttonButton6)) {
                commandID = 14;
            } else if (view == findViewById(R.id.buttonButton7)) {
                commandID = 15;
            } else if (view == findViewById(R.id.buttonReport1)) {
                commandID = 16;
            } else if (view == findViewById(R.id.buttonReport2)) {
                commandID = 17;
            } else if (view == findViewById(R.id.buttonReport3)) {
                commandID = 18;
            } else if (view == findViewById(R.id.buttonParticipant1)) {
                commandID = 19;
            } else if (view == findViewById(R.id.buttonParticipant2)) {
                commandID = 20;
            } else if (view == findViewById(R.id.buttonParticipant3)) {
                commandID = 21;
            } else if (view == findViewById(R.id.buttonParticipant4)) {
                commandID = 22;
            } else {
                Log.d(TAG, "Unknown button");
                return;
            }
            pubs.get(selectedRobot).enqueue(commandID);
            String msg = getTimeStamp() + "," + robotNames[selectedRobot] + "," + commands[commandID].replace('/', ',');
            fw.write(msg + "\n");
            //adapter.add(msg);
            logList.add(msg);
            adapter.notifyDataSetChanged();
            Log.d(TAG, "Wrote: " + msg);
            fw.flush();
            fw.close();
        }
        catch (IOException e)
        {
            Log.d(TAG, e.toString());
        }

    }



    public void rewriteLog(View v) {
        try
        {
            FileWriter fw = new FileWriter(logFile+".rewrite");
            for (int i=0; i<adapter.getCount(); i++)
            {
                fw.write(adapter.getItem(i) + "\n");
            }
        }
        catch (IOException e)
        {
            Log.d(TAG, e.toString());
        }
    }

    public void test(View view)
    {
        Log.d(TAG, "Called from: " + view.getId());
    }
    public void handleRobotSelection(View view) {

        Button b1 = (Button)findViewById(R.id.buttonRobot1Select);
        Button b2 = (Button)findViewById(R.id.buttonRobot2Select);
        Button b3 = (Button)findViewById(R.id.buttonRobot3Select);
        Button b4 = (Button)findViewById(R.id.buttonRobot4Select);
        Button b5 = (Button)findViewById(R.id.buttonRobot5Select);
        Button b6 = (Button)findViewById(R.id.buttonRobot6Select);
        b1.setBackgroundColor(getResources().getColor(R.color.colorDirtdog));
        b2.setBackgroundColor(getResources().getColor(R.color.colorBender));
        b3.setBackgroundColor(getResources().getColor(R.color.colorRoomba));
        b4.setBackgroundColor(getResources().getColor(R.color.colorNeato));
        b5.setBackgroundColor(getResources().getColor(R.color.colorDiscovery));
        b6.setBackgroundColor(getResources().getColor(R.color.colorParticipant));

        if (view == b1)
        {
            view.setBackgroundColor(Color.RED);
            selectedRobot = 0;
            disableParticipantMode();
        }
        else if (view == b2)
        {
            view.setBackgroundColor(Color.RED);
            selectedRobot = 1;
            disableParticipantMode();
        }
        else if (view == b3)
        {
            view.setBackgroundColor(Color.RED);
            selectedRobot = 2;
            disableParticipantMode();
        }
        else if (view == b4)
        {
            view.setBackgroundColor(Color.RED);
            selectedRobot = 3;
            disableParticipantMode();
        }
        else if (view == b5)
        {
            view.setBackgroundColor(Color.RED);
            selectedRobot = 4;
            disableParticipantMode();
        }
        else if (view == b6)
        {
            view.setBackgroundColor(Color.RED);
            selectedRobot = 5;
            enableParticipantMode();
        }
    }

    void enableParticipantMode()
    {
        ArrayList<Button> enableButtons = new ArrayList<>();
        enableButtons.add((Button)findViewById(R.id.buttonGamezone1));
        enableButtons.add((Button)findViewById(R.id.buttonGamezone2));
        enableButtons.add((Button)findViewById(R.id.buttonReport1));
        enableButtons.add((Button)findViewById(R.id.buttonReport2));
        enableButtons.add((Button)findViewById(R.id.buttonReport3));
        enableButtons.add((Button)findViewById(R.id.buttonParticipant1));
        enableButtons.add((Button)findViewById(R.id.buttonParticipant2));
        enableButtons.add((Button)findViewById(R.id.buttonParticipant3));
        enableButtons.add((Button)findViewById(R.id.buttonParticipant4));


        ArrayList<Button> disableButtons = new ArrayList<>();
        disableButtons.add((Button)findViewById(R.id.buttonButton1));
        disableButtons.add((Button)findViewById(R.id.buttonButton2));
        disableButtons.add((Button)findViewById(R.id.buttonButton3));
        disableButtons.add((Button)findViewById(R.id.buttonButton4));
        disableButtons.add((Button)findViewById(R.id.buttonButton5));
        disableButtons.add((Button)findViewById(R.id.buttonButton6));
        disableButtons.add((Button)findViewById(R.id.buttonButton7));
        disableButtons.add((Button)findViewById(R.id.buttonRobot1));
        disableButtons.add((Button)findViewById(R.id.buttonRobot2));
        disableButtons.add((Button)findViewById(R.id.buttonRobot3));
        disableButtons.add((Button)findViewById(R.id.buttonRobot4));
        disableButtons.add((Button)findViewById(R.id.buttonDirtbin1));
        disableButtons.add((Button)findViewById(R.id.buttonDirtbin2));
        disableButtons.add((Button)findViewById(R.id.buttonDirtbin3));

        for (Button b : enableButtons) {
            b.setEnabled(true);
        }
        for (Button b : disableButtons) {
            b.setEnabled(false);
        }
    }

    void disableParticipantMode()
    {
        ArrayList<Button> disableButtons = new ArrayList<>();
        disableButtons.add((Button)findViewById(R.id.buttonGamezone1));
        disableButtons.add((Button)findViewById(R.id.buttonGamezone2));
        disableButtons.add((Button)findViewById(R.id.buttonReport1));
        disableButtons.add((Button)findViewById(R.id.buttonReport2));
        disableButtons.add((Button)findViewById(R.id.buttonReport3));
        disableButtons.add((Button)findViewById(R.id.buttonParticipant1));
        disableButtons.add((Button)findViewById(R.id.buttonParticipant2));
        disableButtons.add((Button)findViewById(R.id.buttonParticipant3));
        disableButtons.add((Button)findViewById(R.id.buttonParticipant4));

        ArrayList<Button> enableButtons = new ArrayList<>();
        enableButtons.add((Button)findViewById(R.id.buttonButton1));
        enableButtons.add((Button)findViewById(R.id.buttonButton2));
        enableButtons.add((Button)findViewById(R.id.buttonButton3));
        enableButtons.add((Button)findViewById(R.id.buttonButton4));
        enableButtons.add((Button)findViewById(R.id.buttonButton5));
        enableButtons.add((Button)findViewById(R.id.buttonButton6));
        enableButtons.add((Button)findViewById(R.id.buttonButton7));
        enableButtons.add((Button)findViewById(R.id.buttonRobot1));
        enableButtons.add((Button)findViewById(R.id.buttonRobot2));
        enableButtons.add((Button)findViewById(R.id.buttonRobot3));
        enableButtons.add((Button)findViewById(R.id.buttonRobot4));
        enableButtons.add((Button)findViewById(R.id.buttonDirtbin1));
        enableButtons.add((Button)findViewById(R.id.buttonDirtbin2));
        enableButtons.add((Button)findViewById(R.id.buttonDirtbin3));

        for (Button b : enableButtons) {
            b.setEnabled(true);
        }
        for (Button b : disableButtons) {
            b.setEnabled(false);
        }
    }
}