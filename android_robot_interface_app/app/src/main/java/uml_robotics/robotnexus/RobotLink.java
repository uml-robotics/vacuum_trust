package uml_robotics.robotnexus;

import android.app.ProgressDialog;
import android.content.Intent;
import android.graphics.Color;
import android.media.Image;
import android.os.Bundle;
import android.os.Handler;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.CardView;
import android.support.v7.widget.Toolbar;
import android.text.Html;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.RelativeLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.SpinnerAdapter;
import android.widget.TextView;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.locks.ReadWriteLock;

/**
 * view for communicating with a particular robot
 */
public class RobotLink extends AppCompatActivity {
    private ArrayList<Robot> model; // view's copy of the model
    private Robot robot = null; // the robot we are linked with
    private ModelUpdate modelUpdate; // responsible for keeping view's model up to date
    private Handler robotLinkHandler; //handler to manipulate robot link UI
    private LinearLayout scrollLayout; // view for appending progressions on
    private ProgressDialog waitDialog; // popup screen for when sending a response to robot
    private ArrayList<Button> activeButtons; // list containing active buttons

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_robot_link);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        model = ControllerService.getModel(); // getting view's copy of the model

        robotLinkHandler = new Handler(); // getting this view's thread

        // getting id of robot given to us
        String robotID = getIntent().getStringExtra("EXTRA_ROBOT_ID");


        // finding robot in our model
        for (Robot bot: model) {
            if (robotID.equals(bot.getId())) {
                robot = bot;
                break;
            }
        }

        // setting title
        setTitle(robot.getName());

        // setting robot's image in toolbar
        ActionBar actionBar = getSupportActionBar(); // getting this activity's actionbar
        ImageView robotImage = new ImageView(actionBar.getThemedContext());
        robotImage.setImageResource(robot.getImage());
        ActionBar.LayoutParams layoutParams = new ActionBar.LayoutParams(
                120, //works for junior image
                170, //works for junior image
                Gravity.END | Gravity.CENTER_VERTICAL);
        layoutParams.rightMargin = 50;
        robotImage.setLayoutParams(layoutParams);
        toolbar.addView(robotImage);


        //set status of robot on UI
        ImageView statusImage = (ImageView)findViewById(R.id.statusIconView);
        String status = robot.getCurrState();
        if (status.equals("ok")) {
            statusImage.setImageResource(R.drawable.svg_ok);
        } else if (status.equals("safe")) {
            statusImage.setImageResource(R.drawable.svg_safe);
        } else if (status.equals("dangerous")) {
            statusImage.setImageResource(R.drawable.svg_dangerous);
        } else if (status.equals("help")) {
            statusImage.setImageResource(R.drawable.svg_help);
        } else if (status.equals("off")) {
            statusImage.setImageResource(R.drawable.svg_off);
        }

        scrollLayout = (LinearLayout)findViewById(R.id.scroll_layout);
        waitDialog = new ProgressDialog(this);
        activeButtons = new ArrayList<>();

        /*
        LayoutInflater inflater =
                (LayoutInflater)getSystemService(LAYOUT_INFLATER_SERVICE);
        RelativeLayout view = (RelativeLayout)inflater.inflate(R.layout.dialog_one, null, false);
        RelativeLayout view2 = (RelativeLayout)inflater.inflate(R.layout.dialog_one, null, false);
        RelativeLayout view3 = (RelativeLayout)inflater.inflate(R.layout.dialog_one, null, false);
        scrollLayout.addView(view);
        scrollLayout.addView(view2);
        scrollLayout.addView(view3);*/

        displayProgression(robot.getProgression());

        // check to see if this activity was started by popup response
        if (getIntent().hasExtra("EXTRA_CHECKSUM")) {
            //now compare checksums to see if robot updated before robot link was created
            if (robot.getStatusHashValue() == getIntent().getLongExtra("EXTRA_CHECKSUM", 0)) {
                // if here then robot has not updated yet and display a waitDialog
                waitDialog.setTitle("Sending message");
                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                waitDialog.setCancelable(false);
                waitDialog.show();

                //disable buttons
                for (Button button : activeButtons) {
                    button.setEnabled(false);
                }
            }
        }

        // check to see if user tapped notification to get here
        if (getIntent().hasExtra("EXTRA_NOTIF_TAPPED")) {
            // if here then log it
            ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                    .format(new Date())
                    + ",NOTIFICATIONTAPPED,"
                    + robot.getName());
        }

        /**
         * MOCK-UP progression dialog
         */
        /*
        // Layout that is the progression box
        RelativeLayout relativeLayout = new RelativeLayout(RobotLink.this);

        // parameters for relative layout
        LinearLayout.LayoutParams relParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT);

        // setting margins for the layout to keep progressions separated
        relParams.setMargins(0, 0, 0, 75);

        // setting parameters for layout
        relativeLayout.setLayoutParams(relParams);

        //changing layouts background color to match theme
        relativeLayout.setBackgroundColor(Color.WHITE);

        // content view of the progression
        TextView contentView = new TextView(RobotLink.this);
        contentView.setText("Would you like me to stop?"); // textual content
        contentView.setTextSize(20); // size of text
        contentView.setTextColor(Color.BLACK); // color of text
        contentView.setId(View.generateViewId()); // generating random id for view

        //parameters for content view
        RelativeLayout.LayoutParams textParams = new RelativeLayout.LayoutParams(
                RelativeLayout.LayoutParams.WRAP_CONTENT,
                RelativeLayout.LayoutParams.WRAP_CONTENT);

        // making text start at left hand side of progression box
        textParams.addRule(RelativeLayout.LEFT_OF);
        textParams.setMargins(20, 30, 0, 30); // margin for content to not be so close to edge
        contentView.setLayoutParams(textParams); // setting parameters for the content view
        relativeLayout.addView(contentView); // appending content into our progression box;


        // making horizontal linear layout to contain buttons
        LinearLayout buttonLayout = new LinearLayout(RobotLink.this);
        buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
        RelativeLayout.LayoutParams buttonLayoutParams = new RelativeLayout.LayoutParams(
                RelativeLayout.LayoutParams.WRAP_CONTENT,
                RelativeLayout.LayoutParams.WRAP_CONTENT
        );
        buttonLayoutParams.addRule(RelativeLayout.CENTER_HORIZONTAL);
        buttonLayoutParams.addRule(RelativeLayout.BELOW, contentView.getId());
        buttonLayoutParams.bottomMargin = 30;
        buttonLayout.setLayoutParams(buttonLayoutParams);


        // Making buttons for each response
        //parameters for button view
        LinearLayout.LayoutParams buttonParams1 = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT);

        buttonParams1.rightMargin = 30;
        LinearLayout.LayoutParams buttonParams2 = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT);

        Button responseButton1 = new Button(RobotLink.this);
        Button responseButton2 = new Button(RobotLink.this);
        responseButton1.setTransformationMethod(null); // remove all caps
        responseButton2.setTransformationMethod(null);
        responseButton1.setText("Yes"); // value of response
        responseButton2.setText("No"); // value of response
        responseButton1.setTextSize(20);
        responseButton2.setTextSize(20);
        responseButton1.setTextColor(Color.BLACK);
        responseButton2.setTextColor(Color.BLACK);
        responseButton1.setBackgroundColor(getResources().getColor(R.color.colorAccent));
        responseButton2.setBackgroundColor(getResources().getColor(R.color.colorAccent));
        responseButton1.setLayoutParams(buttonParams1);
        responseButton2.setLayoutParams(buttonParams2);
        responseButton1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //parameters for response view
                LinearLayout.LayoutParams responseParams = new LinearLayout.LayoutParams(
                        LinearLayout.LayoutParams.WRAP_CONTENT,
                        LinearLayout.LayoutParams.WRAP_CONTENT, Gravity.CENTER_HORIZONTAL);

                responseParams.setMargins(20, 0, 0, 130);
                TextView responseText = new TextView(RobotLink.this);
                responseText.setTextSize(22);
                responseText.setTextColor(Color.BLACK);
                responseText.setLayoutParams(responseParams);
                responseText.setText("Your Response: " + ((Button)v).getText());
                ((LinearLayout)findViewById(R.id.scroll_layout)).addView(responseText);
            }
        });

        responseButton2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //parameters for response view
                LinearLayout.LayoutParams responseParams = new LinearLayout.LayoutParams(
                        LinearLayout.LayoutParams.WRAP_CONTENT,
                        LinearLayout.LayoutParams.WRAP_CONTENT, Gravity.CENTER_HORIZONTAL);

                responseParams.setMargins(20, 0, 0, 130);
                TextView responseText = new TextView(RobotLink.this);
                responseText.setTextSize(22);
                responseText.setTextColor(Color.BLACK);
                responseText.setLayoutParams(responseParams);
                responseText.setText("Your Response: " + ((Button)v).getText());
                ((LinearLayout)findViewById(R.id.scroll_layout)).addView(responseText);
            }
        });
        buttonLayout.addView(responseButton1);
        buttonLayout.addView(responseButton2);



        // appending
        LinearLayout layout = (LinearLayout)findViewById(R.id.scroll_layout);
        relativeLayout.addView(buttonLayout);
        layout.addView(relativeLayout);
        */
        /**
         * END MOCK-UP
         */

    }

    @Override
    protected void onStart() {
        super.onStart();

        // setting progression dialog scroll view to end
        final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
        scrollView.postDelayed(new Runnable() {
            @Override
            public void run() {
                scrollView.fullScroll(View.FOCUS_DOWN);
            }
        }, 75);

        // start model update
        modelUpdate = new ModelUpdate();
        modelUpdate.start();

        /*
        ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                + ": Looking at " + robot.getName());*/

        ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",OPEN,"
                + robot.getName());
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
        modelUpdate.close();
        waitDialog.dismiss();
        ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",CLOSE,"
                + robot.getName());
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    private class ModelUpdate extends Thread {

        private boolean keepAlive = true;
        ArrayList<Robot> modelPrime;
        Robot robotPrime;

        @Override
        public void run() {
            while (keepAlive) {

                modelPrime = ControllerService.getModel();

                for (Robot bot : modelPrime) {
                    if (robot.getId().equals(bot.getId())) {
                        robotPrime = bot;
                        break;
                    }
                }

                // if this robot is now ignored exit activity
                if (!robotPrime.isVisible()) {
                    RobotLink.this.finish();
                    Log.i("RobotLink.Update", "Finishing, " + robotPrime.getId());
                    return;
                }

                // if the robots' progressions or status' don't match up
                if ((!(robotPrime.getCurrState().equals(robot.getCurrState())))
                        || (!(robotPrime.getProgression().toString().equals(robot.getProgression().toString())))) {

                    model = ControllerService.getModel();
                    Log.i("RobotLink.Update", "Model changed");

                    // finding robot in our model
                    for (Robot bot: model) {
                        if (robot.getId().equals(bot.getId())) {
                            robot = bot;
                            robotLinkHandler.post(new Runnable() {
                                @Override
                                public void run() {
                                    //set status of robot on UI
                                    ImageView statusImage = (ImageView)findViewById(R.id.statusIconView);
                                    String status = robot.getCurrState();
                                    if (status.equals("ok")) {
                                        statusImage.setImageResource(R.drawable.svg_ok);
                                    } else if (status.equals("safe")) {
                                        statusImage.setImageResource(R.drawable.svg_safe);
                                    } else if (status.equals("dangerous")) {
                                        statusImage.setImageResource(R.drawable.svg_dangerous);
                                    } else if (status.equals("help")) {
                                        statusImage.setImageResource(R.drawable.svg_help);
                                    } else if (status.equals("off")) {
                                        statusImage.setImageResource(R.drawable.svg_off);
                                    }

                                    scrollLayout.removeAllViews();
                                    displayProgression(robot.getProgression());
                                }
                            });
                            break;
                        }
                    }
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




    private void displayProgression(JSONArray progression) {



        //if (progression == null) {
        //    return;
        //}

        //clear old buttons from list
        activeButtons.clear();

        /**
         * loop through all objects in progression
         */
        for (int i = 0; i < progression.length(); i++) {
            try {

                // object for using layout files
                final LayoutInflater inflater =
                        (LayoutInflater)getSystemService(LAYOUT_INFLATER_SERVICE);

                RelativeLayout dialogBox = null;

                RelativeLayout responseText = null;

                // get this element in the progression
                final JSONObject progressionElement = progression.getJSONObject(i);

                // get responses which will be turned into buttons
                int numOfResponses = 0;
                JSONArray responses = null;
                if (!progressionElement.isNull("responses")) {
                    responses = progressionElement.getJSONArray("responses");
                    numOfResponses = responses.length();
                }

                // use appropriate layout on responses length
                if (numOfResponses == 0
                        || (i+1) != progression.length()
                        || (!progressionElement.isNull("selection"))) {
                    // no responses received or don't give user buttons to old prog elements
                    dialogBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);

                    //set content
                    TextView contentView = (TextView) dialogBox.findViewById(R.id.dialog_zero_content);
                    contentView.setText(Html.fromHtml(progressionElement.getString("content")));

                    // if true then the user has already responded to this prog element
                    if (!progressionElement.isNull("selection")) {
                        String responseId = progressionElement.getString("selection");
                        for (int j = 0; j < numOfResponses; j++) {
                            if (responseId.equals(responses.getJSONObject(j).getString("id"))) {
                                //set as custom layout
                                responseText = (RelativeLayout)inflater
                                        .inflate(R.layout.dialog_response_dropdown, null, false);
                                // get the spinner
                                final Spinner dropdown = (Spinner)responseText.findViewById(R.id.dialog_dropdown);
                                // list for the spinner
                                // source before modifications:
                                // http://coding-thoughts.blogspot.com/2013/11/help-my-spinner-is-too-wide.html
                                final ArrayAdapter<String> dropResponses = new ArrayAdapter<String>(RobotLink.this,
                                        R.layout.text_resource) {

                                    @Override
                                    public View getView(final int position, final View convertView,
                                                        final ViewGroup parent) {
                                        int selectedItemPosition = position;
                                        if (parent instanceof AdapterView) {
                                            selectedItemPosition = ((AdapterView) parent)
                                                    .getSelectedItemPosition();
                                        }
                                        return makeLayout(selectedItemPosition, convertView, parent,
                                                R.layout.text_resource);
                                    }

                                    @Override
                                    public View getDropDownView(final int position, final View convertView,
                                                                final ViewGroup parent) {
                                        return makeLayout(position, convertView, parent,
                                                android.R.layout.simple_spinner_dropdown_item);
                                    }

                                    private View makeLayout(final int position, final View convertView,
                                                            final ViewGroup parent, final int layout) {
                                        TextView tv;
                                        if (convertView != null) {
                                            tv = (TextView) convertView;
                                        } else {
                                            tv = (TextView) LayoutInflater.from(RobotLink.this).inflate(layout,
                                                    parent, false);
                                        }

                                        tv.setText(this.getItem(position));

                                        return tv;
                                    }

                                };
                                for (int k = 0; k < numOfResponses; k++) {
                                    dropResponses.add(responses.getJSONObject(k).getString("value"));
                                }


                                //dropResponses.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                                dropdown.setAdapter(dropResponses);
                                dropdown.setSelection(j, false);
                                dropdown.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                                    @Override
                                    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                                        try {

                                            String responseValue = dropResponses.getItem(position);
                                            JSONArray responses = progressionElement.getJSONArray("responses");
                                            for (int i = 0; i < responses.length(); i++) {
                                                if (responseValue.
                                                        equals(responses.getJSONObject(i).getString("value"))) {

                                                    ControllerService.addToReplyQueue(robot.getId(),
                                                            progressionElement.getString("msgid"),
                                                            responses.getJSONObject(i).getString("id"));
                                                    break;
                                                }
                                            }

                                            //show waiting dialog
                                            waitDialog.setTitle("Sending message");
                                            waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                            waitDialog.setCancelable(false);
                                            waitDialog.show();

                                            //disable buttons
                                            for (Button button : activeButtons) {
                                                button.setEnabled(false);
                                            }

                                            /*
                                            ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                                    + ": Sent \"" + responseValue  + "\" to "
                                                    + robot.getName() + " in response to \""
                                                    + progressionElement.getString("content")
                                                    + "\"");
                                            */

                                            ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                                    .format(new Date())
                                                    + ",SELECTED,"
                                                    + responseValue);

                                        } catch (JSONException ex) {
                                            StringWriter stringWriter = new StringWriter();
                                            PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                            ex.printStackTrace(printWriter);
                                            Log.e("RobotLink.Progression", stringWriter.toString());
                                        }
                                    }

                                    @Override
                                    public void onNothingSelected(AdapterView<?> parent) {

                                    }
                                });
                                break;
                            }
                        }
                    }

                } else if (numOfResponses == 1) {
                    // 1 button
                    dialogBox = (RelativeLayout)inflater.inflate(R.layout.dialog_one, null, false);

                    //set content
                    TextView contentView = (TextView) dialogBox.findViewById(R.id.dialog_one_content);
                    contentView.setText(Html.fromHtml(progressionElement.getString("content")));

                    //set button values
                    final Button button1 = (Button) dialogBox.findViewById(R.id.dialog_one_button1);
                    final JSONObject responseElement1 = responses.getJSONObject(0);
                    button1.setText(responseElement1.getString("value"));
                    button1.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement1.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling button
                                button1.setEnabled(false);

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Sent \"" + responseElement1.getString("value")
                                        + "\" to " + robot.getName() + " in response to \""
                                        + progressionElement.getString("content")
                                        + "\"");
                                */

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement1.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }
                        }
                    });

                    activeButtons.add(button1);

                } else if (numOfResponses == 2) {
                    // 2 buttons
                    dialogBox = (RelativeLayout)inflater.inflate(R.layout.dialog_two, null, false);

                    //set content
                    TextView contentView = (TextView) dialogBox.findViewById(R.id.dialog_two_content);
                    contentView.setText(Html.fromHtml(progressionElement.getString("content")));

                    //set button values
                    final Button button1 = (Button) dialogBox.findViewById(R.id.dialog_two_button1);
                    final Button button2 = (Button) dialogBox.findViewById(R.id.dialog_two_button2);
                    final JSONObject responseElement1 = responses.getJSONObject(0);
                    final JSONObject responseElement2 = responses.getJSONObject(1);
                    button1.setText(responseElement1.getString("value"));
                    button2.setText(responseElement2.getString("value"));
                    button1.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement1.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                // setting progression dialog scroll view to end
                                /*
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement1.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }
                        }
                    });
                    button2.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement2.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement2.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }

                        }
                    });
                    activeButtons.add(button1);
                    activeButtons.add(button2);

                } else if (numOfResponses == 3) {
                    // 3 buttons
                    dialogBox = (RelativeLayout)inflater.inflate(R.layout.dialog_three, null, false);

                    //set content
                    TextView contentView = (TextView) dialogBox.findViewById(R.id.dialog_three_content);
                    contentView.setText(Html.fromHtml(progressionElement.getString("content")));

                    //set button values
                    final Button button1 = (Button) dialogBox.findViewById(R.id.dialog_three_button1);
                    final Button button2 = (Button) dialogBox.findViewById(R.id.dialog_three_button2);
                    final Button button3 = (Button) dialogBox.findViewById(R.id.dialog_three_button3);
                    final JSONObject responseElement1 = responses.getJSONObject(0);
                    final JSONObject responseElement2 = responses.getJSONObject(1);
                    final JSONObject responseElement3 = responses.getJSONObject(2);
                    button1.setText(responseElement1.getString("value"));
                    button2.setText(responseElement2.getString("value"));
                    button3.setText(responseElement3.getString("value"));
                    button1.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement1.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement1.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }
                        }
                    });
                    button2.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement2.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement2.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }

                        }
                    });
                    button3.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement3.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement3.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }

                        }
                    });

                    activeButtons.add(button1);
                    activeButtons.add(button2);
                    activeButtons.add(button3);

                } else if (numOfResponses == 4) {
                    // 4 buttons
                    dialogBox = (RelativeLayout)inflater.inflate(R.layout.dialog_four, null, false);

                    //set content
                    TextView contentView = (TextView) dialogBox.findViewById(R.id.dialog_four_content);
                    contentView.setText(Html.fromHtml(progressionElement.getString("content")));

                    //set button values
                    final Button button1 = (Button) dialogBox.findViewById(R.id.dialog_four_button1);
                    final Button button2 = (Button) dialogBox.findViewById(R.id.dialog_four_button2);
                    final Button button3 = (Button) dialogBox.findViewById(R.id.dialog_four_button3);
                    final Button button4 = (Button) dialogBox.findViewById(R.id.dialog_four_button4);
                    final JSONObject responseElement1 = responses.getJSONObject(0);
                    final JSONObject responseElement2 = responses.getJSONObject(1);
                    final JSONObject responseElement3 = responses.getJSONObject(2);
                    final JSONObject responseElement4 = responses.getJSONObject(3);
                    button1.setText(responseElement1.getString("value"));
                    button2.setText(responseElement2.getString("value"));
                    button3.setText(responseElement3.getString("value"));
                    button4.setText(responseElement4.getString("value"));
                    button1.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement1.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);
                                button4.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement1.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }
                        }
                    });
                    button2.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement2.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);
                                button4.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement2.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }

                        }
                    });
                    button3.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement3.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/

                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);
                                button4.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement3.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }

                        }
                    });
                    button4.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(robot.getId(),
                                        progressionElement.getString("msgid"), responseElement4.getString("id"));

                                //display waiting box
                                /*
                                RelativeLayout waitBox = (RelativeLayout)inflater.inflate(R.layout.dialog_zero, null, false);
                                TextView contentView = (TextView) waitBox.findViewById(R.id.dialog_zero_content);
                                contentView.setText("Waiting for robot...");
                                scrollLayout.addView(waitBox);

                                // setting progression dialog scroll view to end
                                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                                scrollView.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        scrollView.fullScroll(View.FOCUS_DOWN);
                                    }
                                });*/
                                //show waiting dialog
                                waitDialog.setTitle("Sending message");
                                waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                                waitDialog.setCancelable(false);
                                waitDialog.show();

                                //disabling buttons
                                button1.setEnabled(false);
                                button2.setEnabled(false);
                                button3.setEnabled(false);
                                button4.setEnabled(false);

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responseElement4.getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("RobotLink.Progression", stringWriter.toString());
                            }

                        }
                    });

                    activeButtons.add(button1);
                    activeButtons.add(button2);
                    activeButtons.add(button3);
                    activeButtons.add(button4);
                }

                scrollLayout.addView(dialogBox);

                //if there was a response to this progression element then show it
                if (responseText != null) {
                    scrollLayout.addView(responseText);
                }

                // setting progression dialog scroll view to end
                final ScrollView scrollView = ((ScrollView) findViewById(R.id.scroll_view));
                scrollView.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        scrollView.fullScroll(View.FOCUS_DOWN);
                    }
                }, 75);


                // disable waiting dialog if it showing
                waitDialog.dismiss();

            } catch (JSONException ex) {
                StringWriter stringWriter = new StringWriter();
                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                ex.printStackTrace(printWriter);
                Log.e("RobotLink.Progression", stringWriter.toString());
            }
        }

//            try {
//                // get this element in the progression
//                final JSONObject progressionElement = progression.getJSONObject(i);
//
//                //display the content
//                TextView contentView = new TextView(RobotLink.this);
//                contentView.setText(Html.fromHtml(progressionElement.getString("content"))); // textual content
//                contentView.setTextSize(20); // size of text
//                contentView.setTextColor(Color.BLACK); // color of text
//                contentView.setId(View.generateViewId()); // generating random id for view
//
//                //parameters for content view
//                LinearLayout.LayoutParams textParams = new LinearLayout.LayoutParams(
//                        LinearLayout.LayoutParams.WRAP_CONTENT,
//                        LinearLayout.LayoutParams.WRAP_CONTENT);
//
//                textParams.setMargins(20, 50, 0, 10); // margin for content to not be so close to edge
//                contentView.setLayoutParams(textParams); // setting parameters for the content view
//
//                scrollLayout.addView(contentView);
//
//                // get responses which will be turned into buttons
//                JSONArray responses = progressionElement.getJSONArray("responses");
//
//
//                // Layout that is the progression box
//                RelativeLayout relativeLayout = new RelativeLayout(RobotLink.this);
//
//                // parameters for relative layout
//                LinearLayout.LayoutParams relParams = new LinearLayout.LayoutParams(
//                        LinearLayout.LayoutParams.MATCH_PARENT,
//                        LinearLayout.LayoutParams.WRAP_CONTENT);
//
//                // setting margins for the layout to keep progressions separated
//                relParams.setMargins(0, 0, 0, 75);
//
//                // setting parameters for layout
//                relativeLayout.setLayoutParams(relParams);
//
//                //changing layouts background color to match theme
//                relativeLayout.setBackgroundColor(Color.WHITE);
//
//                // making horizontal linear layout to contain buttons
//                LinearLayout buttonLayout = new LinearLayout(RobotLink.this);
//                buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
//                RelativeLayout.LayoutParams buttonLayoutParams = new RelativeLayout.LayoutParams(
//                        RelativeLayout.LayoutParams.WRAP_CONTENT,
//                        RelativeLayout.LayoutParams.WRAP_CONTENT
//                );
//                buttonLayoutParams.addRule(RelativeLayout.CENTER_HORIZONTAL);
//                buttonLayout.setLayoutParams(buttonLayoutParams);
//
//                /**
//                 * loop through all responses and make buttons for them
//                 */
//                for (int j = 0; j < responses.length(); j++) {
//                    // get this element in the response
//                    final JSONObject responseElement = responses.getJSONObject(j);
//
//                    //parameters for button view
//                    LinearLayout.LayoutParams buttonParams1 = new LinearLayout.LayoutParams(
//                            LinearLayout.LayoutParams.WRAP_CONTENT,
//                            LinearLayout.LayoutParams.WRAP_CONTENT);
//                    buttonParams1.rightMargin = 10;
//                    Button responseButton1 = new Button(RobotLink.this);
//                    responseButton1.setTransformationMethod(null); // remove all caps
//                    responseButton1.setText(responseElement.getString("value")); // value of response
//                    responseButton1.setTextSize(20);
//                    responseButton1.setTextColor(Color.BLACK);
//                    responseButton1.setBackgroundColor(getResources().getColor(R.color.colorAccent));
//                    responseButton1.setLayoutParams(buttonParams1);
//                    responseButton1.setOnClickListener(new View.OnClickListener() {
//                        @Override
//                        public void onClick(View v) {
//                            try {
//                                ControllerService.addToReplyQueue(robot.getId(),
//                                        progressionElement.getString("msgid"), responseElement.getString("id"));
//                            } catch (JSONException ex) {
//                                StringWriter stringWriter = new StringWriter();
//                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
//                                ex.printStackTrace(printWriter);
//                                Log.e("RobotLink.Progression", stringWriter.toString());
//                            }
//                        }
//                    });
//                    buttonLayout.addView(responseButton1);
//                }
//
//                scrollLayout.addView(buttonLayout);
//
//            } catch (JSONException ex) {
//                StringWriter stringWriter = new StringWriter();
//                PrintWriter printWriter = new PrintWriter(stringWriter, true);
//                ex.printStackTrace(printWriter);
//                Log.e("RobotLink.Progression", stringWriter.toString());
//            }
//
//        }
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        // getting id of robot given to us
        String robotID = intent.getStringExtra("EXTRA_ROBOT_ID");
        if (robotID.equals(robot.getId())) {

            // check to see if this activity was started by popup response
            if (intent.hasExtra("EXTRA_CHECKSUM")) {

                //now compare checksums to see if robot updated before robot link was created
                if (robot.getStatusHashValue() == intent.getLongExtra("EXTRA_CHECKSUM", 0)) {

                    // if here then robot has not updated yet and display a waitDialog
                    waitDialog.setTitle("Sending message");
                    waitDialog.setMessage("Waiting for " + robot.getName() + "..." );
                    waitDialog.setCancelable(false);
                    waitDialog.show();

                    //disable buttons
                    for (Button button : activeButtons) {
                        button.setEnabled(false);
                    }
                }
            }

            // check to see if user tapped notification to get here
            if (intent.hasExtra("EXTRA_NOTIF_TAPPED")) {
                // if here then log it
                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                        .format(new Date())
                        + ",NOTIFICATIONTAPPED,"
                        + robot.getName());
            }

            // this is the same robot -> do not recreate activity
            return;
        }
        startActivity(intent);
        this.finish();
    }
}
