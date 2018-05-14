package uml_robotics.robotnexus;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ListView;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;

public class RobotSelector extends AppCompatActivity {
    private ArrayList<Robot> model; // view's copy of the model
    private ModelUpdate modelUpdate; // responsible for keeping view's model up to date
    private RobotNavListAdapter displayAdapter; //adapter for listview to display robot info
    private Handler robotSelectorHandler; //handler to manipulate robot selector UI

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_robot_selector);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);


        model = ControllerService.getModel(); // getting view's copy of the model

        robotSelectorHandler = new Handler(); // getting this view's thread

        //setting up robot list ui
        /*
        Integer images[]  = new Integer[model.size()];
        Robot robots[] = new Robot[model.size()];
        int i = 0;
        for (Robot bot : model) {
            robots[i] = bot;
            images[i] = bot.getImage();
            i++;
        }
        displayAdapter = new RobotNavListAdapter(RobotSelector.this, robots, images);*/

        // checking how many robots needs to be displayed (not ignored)
        int robotListSize = 0;
        for (Robot bot : model) {
            if (bot.isVisible()) {
                robotListSize++;
            }
        }

        //setting up robot list ui
        Integer images[]  = new Integer[robotListSize];
        Robot robots[] = new Robot[robotListSize];
        Integer icons[] = new Integer[robotListSize];
        int i = 0;
        for (Robot bot : model) {
            if (!bot.isVisible()) {
                continue;
            }
            robots[i] = bot;
            images[i] = bot.getImage();
            switch (bot.getCurrState()) {
                case "ok":
                    icons[i] = R.drawable.svg_ok;
                    break;
                case "safe":
                    icons[i] = R.drawable.svg_safe;
                    break;
                case "dangerous":
                    icons[i] = R.drawable.svg_dangerous;
                    break;
                case "help":
                    icons[i] = R.drawable.svg_help;
                    break;
                case "off":
                    icons[i] = R.drawable.svg_off;
                    break;
            }
            i++;
            Log.i("RobotSelector.onCreate", "ID: " + bot.getId());
            Log.i("RobotSelector.onCreate", "Name: " + bot.getName());
            Log.i("RobotSelector.onCreate", "ImgID: " + bot.getImage());
        }
        Log.i("RobotSelector.onCreate", "Res ImgIDs: " + Arrays.toString(images));
        Log.i("RobotSelector.onCreate", "Robot Names: " + Arrays.toString(robots));
        displayAdapter = new RobotNavListAdapter(RobotSelector.this, robots, images, icons);

        ListView listView = (ListView) findViewById(R.id.robot_list);
        listView.setAdapter(displayAdapter);

        // enable click event handling
        listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                //ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                //        + ": Selected " + displayAdapter.getItem(position).getName() +
                //        " from nearby robots");
                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                        .format(new Date())
                        + ",SELECTED,"
                        + displayAdapter.getItem(position).getName());
                Intent intent = new Intent(RobotSelector.this, RobotLink.class);
                intent.putExtra("EXTRA_ROBOT_ID", displayAdapter.getItem(position).getId());
                startActivity(intent);
            }
        });
    }

    @Override
    protected void onStart() {
        super.onStart();
        modelUpdate = new ModelUpdate();
        modelUpdate.start();
        ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",OPEN,nearby_robots");
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onStop() {
        super.onStop();
        // end our update
        modelUpdate.close();
        ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",CLOSE,nearby_robots");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    private class ModelUpdate extends Thread {

        private boolean keepAlive = true;
        ArrayList<Robot> modelPrime;

        @Override
        public void run() {
            while (keepAlive) {

                modelPrime = ControllerService.getModel();
                /*
                for (Robot robot : modelPrime) {
                    Log.i("RobotSelector.Update", "Prime-" + robot.getId() + ": " + robot.isVisible());
                }
                for (Robot robot : model) {
                    Log.i("RobotSelector.Update", "Self-" + robot.getId() + ": " + robot.isVisible());
                }*/

                // if our models don't match up
                if (!(modelPrime.containsAll(model) && model.containsAll(modelPrime))) {

                    model = ControllerService.getModel();
                    Log.i("RobotSelector.Update", "Model changed");

                    // checking how many robots needs to be displayed (not ignored)
                    int robotListSize = 0;
                    for (Robot bot : model) {
                        if (bot.isVisible()) {
                            robotListSize++;
                        }
                    }

                    //updating robot list ui
                    Integer images[]  = new Integer[robotListSize];
                    Robot robots[] = new Robot[robotListSize];
                    Integer icons[] = new Integer[robotListSize];
                    int i = 0;
                    for (Robot bot : model) {
                        if (!bot.isVisible()) {
                            continue;
                        }
                        robots[i] = bot;
                        images[i] = bot.getImage();
                        switch (bot.getCurrState()) {
                            case "ok":
                                icons[i] = R.drawable.svg_ok;
                                break;
                            case "safe":
                                icons[i] = R.drawable.svg_safe;
                                break;
                            case "dangerous":
                                icons[i] = R.drawable.svg_dangerous;
                                break;
                            case "help":
                                icons[i] = R.drawable.svg_help;
                                break;
                            case "off":
                                icons[i] = R.drawable.svg_off;
                                break;
                        }
                        i++;
                        Log.i("RobotSelector.Update", "ID: " + bot.getId());
                        Log.i("RobotSelector.Update", "Name: " + bot.getName());
                        Log.i("RobotSelector.Update", "ImgID: " + bot.getImage());
                    }
                    Log.i("RobotSelector.Update", "Res ImgIDs: " + Arrays.toString(images));
                    Log.i("RobotSelector.Update", "Robot Names: " + Arrays.toString(robots));
                    displayAdapter = new RobotNavListAdapter(RobotSelector.this, robots, images, icons);

                    robotSelectorHandler.post(new Runnable() {
                        @Override
                        public void run() {
                            ListView listView = (ListView) findViewById(R.id.robot_list);
                            listView.setAdapter(displayAdapter);
                            displayAdapter.notifyDataSetChanged();
                            //Toast.makeText(RobotSelector.this, "Update", Toast.LENGTH_SHORT).show();
                        }
                    });

                }

                try {
                    sleep(300);
                } catch (InterruptedException ex) {

                }

            }
        }

        public void close() {
            keepAlive = false;
        }
    }

    // back button from robot selector sends user to home screen instead of hidden main activity
    @Override
    public void onBackPressed() {
        //super.onBackPressed();
        //ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
        //        + ": No longer looking at nearby robots");
        Intent startMain = new Intent(Intent.ACTION_MAIN);
        startMain.addCategory(Intent.CATEGORY_HOME);
        startMain.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        startActivity(startMain);
    }
}
