package uml_robotics.robotnexus;

import android.app.Dialog;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.media.Ringtone;
import android.media.RingtoneManager;
import android.net.Uri;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.os.Vibrator;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.NotificationCompat;
import android.text.Html;
import android.text.LoginFilter;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.RemoteViews;
import android.widget.TextView;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.zip.CRC32;

import static java.lang.Thread.sleep;

public class NotificationViewService extends Service {
    private NotificationManager notifManager; // manager object for notification related events
    private ArrayList<Robot> model; // view's copy of the model
    private ModelUpdate modelUpdate; // responsible for keeping view's model up to date
    private Dialog dialog = null; // emergency dialog for popups
    private Notification.Builder notif; // builder object for notifications
    // used for making sure the same dialog is not displayed twice
    //private HashMap<String, String> botsThatMessaged;

    public NotificationViewService() {}

    @Override
    public void onCreate() {
        Log.i("NotifView.onCreate()", "Service Created");

        notifManager = (NotificationManager)getSystemService(Context.NOTIFICATION_SERVICE);
        notif = new Notification.Builder(NotificationViewService.this);
        //botsThatMessaged = new HashMap<>();
        model = ControllerService.getModel();

        // starting model update
        modelUpdate = new ModelUpdate();
        modelUpdate.start();


    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return Service.START_STICKY;
    }

    @Override
    public void onDestroy() {
        // clear all notifications
        notifManager.cancelAll();
        modelUpdate.close();
        Log.i("NotifView.onDestroy()", "Destroyed");
    }

    /**
     *returns null -> this is a pure started service
     */
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    private class ModelUpdate extends Thread {

        private boolean keepAlive = true;
        ArrayList<Robot> modelPrime;

        @Override
        public void run() {
            while (keepAlive) {

                modelPrime = ControllerService.getModel();

                // if our models don't match up
                if (!(modelPrime.containsAll(model) && model.containsAll(modelPrime))) {

                    // used to notify only if checksum is different
                    ArrayList<Robot> oldModel = new ArrayList<>();
                    for (Robot robot: model) {
                        oldModel.add((Robot)robot.clone());
                    }

                    model = ControllerService.getModel();
                    Log.i("NotifView.Update", "Model changed");

                    //reverse this copy of the model to ensure closest robot is the top notif
                    //Collections.reverse(model);

                    // ***displaying push notifications***

                    int id = 0; // for content intent and notification
                    //ArrayList<Notification> theNotifications = new ArrayList<>();
                    //CRC32 crc32 = new CRC32();
                    for (Robot bot : model) {

                        // getting id for this bot's notification
                        //crc32.reset();
                        //crc32.update(bot.getId().getBytes());
                        //int id = (int)crc32.getValue();

                        // if this robot is leaving cancel its notification
                        if (!bot.isVisible()) {
                            notifManager.cancel(id);
                            id++;
                            Log.i("NotifView.Update", "Canceling: " + bot.getId());
                            continue;
                        }

                        // check to see if robot changed at all
                        boolean hasUpdate = true;
                        for (Robot robot: oldModel) {
                            if (robot.getId().equals(bot.getId())) {
                                if (robot.getStatusHashValue() == bot.getStatusHashValue()) {
                                    // this robot did not update
                                    hasUpdate = false;
                                    Log.i("NotifView.Update", "Old model: " + robot.getId());
                                    Log.i("NotifView.Update", "New model: " + bot.getId());
                                    Log.i("NotifView.Update", "Old model: " + robot.getStatusHashValue());
                                    Log.i("NotifView.Update", "New model: " + bot.getStatusHashValue());
                                    break;
                                }
                            }
                        }
                        if (!hasUpdate) {
                            id++;
                            continue;
                        }

                        Log.i("NotifView.Update", "Notifying: " + bot.getId());

                        // building notification
                        // get our custom notification layout
                        RemoteViews remoteViews = new RemoteViews(getPackageName(), R.layout.custom_push_notification);

                        // set the img of the robot
                        if (bot.getModel() == null) {
                            // safety-net for ack
                            remoteViews.setImageViewResource(R.id.push_notif_img, R.mipmap.ic_launcher);
                        } else {
                            remoteViews.setImageViewResource(R.id.push_notif_img, bot.getImage());
                        }

                        //setting robot's name
                        remoteViews.setTextViewText(R.id.push_notif_bot_name, bot.getName());

                        // setting icon of notification
                        if (bot.getImage() == null) {
                            notif.setSmallIcon(R.mipmap.ic_launcher);
                        } else {
                            notif.setSmallIcon(bot.getImage());
                        }

                        // setting of status of robot
                        switch (bot.getCurrState()) {
                            case "ok":
                                remoteViews.setImageViewResource(R.id.push_notif_status, R.drawable.svg_ok);
                                break;
                            case "safe":
                                remoteViews.setImageViewResource(R.id.push_notif_status, R.drawable.svg_safe);
                                break;
                            case "dangerous":
                                remoteViews.setImageViewResource(R.id.push_notif_status, R.drawable.svg_dangerous);
                                break;
                            case "help":
                                remoteViews.setImageViewResource(R.id.push_notif_status, R.drawable.svg_help);
                                break;
                            case "off":
                                remoteViews.setImageViewResource(R.id.push_notif_status, R.drawable.svg_off);
                                break;
                        }

                        notif.setContent(remoteViews);

                        //make notification persistent
                        //notif.setOngoing(true);

                        //notif.setLargeIcon(BitmapFactory.decodeResource(getResources(), bot.getImage()));


                        // setting title of notification
                        //notif.setContentTitle(bot.getName());

                        // setting textual content of notification - when notif isnt expanded
                        //notif.setContentText("Autonomous system is nearby.");

                        // Big style notification text
                        //notif.setStyle((new Notification.BigTextStyle()).bigText(
                                //"Autonomous system is nearby.\nTap for more information"
                        //));

                        //notif.setStyle(new NotificationCompat.BigPictureStyle().bigPicture(
                          //      BitmapFactory.decodeResource(getResources(), R.drawable.dangerous)
                        //).setSummaryText("Autonomous system is nearby.\nTap for more information"));


                        // setting clickable action of notification
                        Intent intent = new Intent(NotificationViewService.this, RobotLink.class);
                        intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
                        intent.putExtra("EXTRA_ROBOT_ID", bot.getId());
                        intent.putExtra("EXTRA_NOTIF_TAPPED", true);
                        notif.setContentIntent(PendingIntent.getActivity(NotificationViewService.this,
                                id, intent,
                                PendingIntent.FLAG_UPDATE_CURRENT));


                        // Adding dismiss button
                        /*
                        Intent dismissIntent = new Intent();
                        dismissIntent.setAction(DISMISS);
                        notif.addAction(R.drawable.dismiss, "Dismiss", PendingIntent.getBroadcast(NotificationViewService.this,
                                0, dismissIntent, PendingIntent.FLAG_UPDATE_CURRENT));
                        */

                        notifManager.notify(id, notif.build());
                        //Log.i("NotifView", "ID: " + id + "Bot: " + bot.getName());

                        //theNotifications.add(notif.build());
                        //closestThree.add(notif.build());
                        id++;
                        //if (id == 3) {
                        //    break;
                        //}
                    }

                    // notify
                    //for (int i = (theNotifications.size()-1); i > -1; i--) {
                       // notifManager.notify(theNotifications.remove(i));
                    //}
                    //for (int i = 0; i < theNotifications.size(); i++) {
                    //        notifManager.notify(i, theNotifications.get(i));
                    //}

                    // *** end displaying push notifications ***

                    // ***displaying emergency dialog (pull notifications)***
                    for (final Robot bot : model) {

                        // if this robot is not visible don't display a popup even if it has one
                        if (!bot.isVisible()) {
                            continue;
                        }

                        try {

                            JSONArray progression = bot.getProgression();
                            // make sure this bot has a progression
                            if (progression.length() > 0) {

                                // get the last dialog in the progression
                                final JSONObject lastProgressionElement = progression.getJSONObject(progression.length() - 1);

                                // check its popup field
                                if (lastProgressionElement.getBoolean("popup")
                                        //|| progression.length() == 1
                                        ) {

                                    //if (botsThatMessaged.containsKey(bot.getId())) {
                                        // check to see if we already messaged user
                                        //if (!(botsThatMessaged.get(bot.getId())
                                        //        .equals(lastProgressionElement.getString("msgid")))) {
                                            // if here then make this progression element an emergency dialog
                                            new Handler(Looper.getMainLooper()).post(new Runnable() {
                                                @Override
                                                public void run() {
                                                    if (dialog == null || !dialog.isShowing()) {
                                                        displayDialog(bot, lastProgressionElement);
                                                    }
                                                }
                                            });
                                            break; // just show one emergency dialog at a time
                                    //   }
                                   //} else {
                                    /*
                                        // if here then make this progression element an emergency dialog
                                        new Handler(Looper.getMainLooper()).post(new Runnable() {
                                            @Override
                                            public void run() {
                                                if (dialog == null || !dialog.isShowing()) {
                                                    displayDialog(bot, lastProgressionElement);
                                                }
                                            }
                                        });
                                        break; // just show one emergency dialog at a time
                                    } */
                                }
                            }

                        } catch (JSONException ex) {
                            StringWriter stringWriter = new StringWriter();
                            PrintWriter printWriter = new PrintWriter(stringWriter, true);
                            ex.printStackTrace(printWriter);
                            Log.e("NotifView.Update", stringWriter.toString());
                        }
                    }
                    // *** end displaying emergency dialog ***
                }

                try {
                    sleep(450);
                } catch (InterruptedException ex) {

                }
            }
        }
        public void close() {
            keepAlive = false;
        }
    }

    public void displayDialog(final Robot bot, final JSONObject progressionElement) {


        // creating dialog
        dialog = new Dialog(NotificationViewService.this);
        // allowing to write over an screen
        dialog.getWindow().setType(WindowManager.LayoutParams.TYPE_SYSTEM_ALERT);
        dialog.requestWindowFeature(Window.FEATURE_LEFT_ICON);
        // setting title of dialog
        dialog.setTitle( "Message from " + bot.getName());

        dialog.setCanceledOnTouchOutside(false); //USE THIS

        try {
            // get responses (buttons
            final JSONArray responses = progressionElement.getJSONArray("responses");

            // status of robot to display
            String status = bot.getCurrState();

            // figure out which layout to use
            switch (responses.length()) {

                // 2 buttons
                case 2:
                    dialog.setContentView(R.layout.alert_two);
                    Button button1 = ((Button)dialog.findViewById(R.id.alert_two_button1));
                    Button button2 = ((Button)dialog.findViewById(R.id.alert_two_button2));

                    button1.setText(responses.getJSONObject(0).getString("value"));
                    button2.setText(responses.getJSONObject(1).getString("value"));

                    // clickable actions for buttons -> add to controller reply queue
                    button1.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(0).getString("id"));
                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(0).getString("value"));
                                        */

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Sent \"" + responses.getJSONObject(0).getString("value")
                                        + "\" to " + bot.getName() + " in response to a popup saying \""
                                        + progressionElement.getString("content")
                                        + "\"");
                                        */
                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(0).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    button2.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(1).getString("id"));

                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(1).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(1).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    // set status of robot
                    if (status.equals("ok")) {
                        ((ImageView)dialog.findViewById(R.id.alert_two_img)).setImageResource(R.drawable.svg_ok);
                    } else if (status.equals("safe")) {
                        ((ImageView)dialog.findViewById(R.id.alert_two_img)).setImageResource(R.drawable.svg_safe);
                    } else if (status.equals("dangerous")) {
                        ((ImageView)dialog.findViewById(R.id.alert_two_img)).setImageResource(R.drawable.svg_dangerous);
                    } else if (status.equals("help")) {
                        ((ImageView)dialog.findViewById(R.id.alert_two_img)).setImageResource(R.drawable.svg_help);
                    } else if (status.equals("off")) {
                        ((ImageView)dialog.findViewById(R.id.alert_two_img)).setImageResource(R.drawable.svg_off);
                    }

                    // set content of progression
                    ((TextView)dialog.findViewById(R.id.alert_two_content))
                            .setText(Html.fromHtml(progressionElement.getString("content")));

                    // setting clickable action for later and more info
                    ((Button)dialog.findViewById(R.id.alert_two_info)).setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                    RobotLink.class)
                                    .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                    .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));
                            dialog.dismiss();

                            ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                    .format(new Date())
                                    + ",SELECTED,more_info");
                        }
                    });

                    /*
                    ((Button)dialog.findViewById(R.id.alert_two_later)).setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            //get rid of dialog box and don't show again for this robot
                            dialog.dismiss();

                            ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                    + ": Selected later on " + bot.getName()
                                    + " from popup");
                        }
                    });*/

                    break;
                // 3 buttons
                case 3:

                    dialog.setContentView(R.layout.alert_three);
                    Button button3 = ((Button)dialog.findViewById(R.id.alert_three_button1));
                    Button button4 = ((Button)dialog.findViewById(R.id.alert_three_button2));
                    Button button5 = ((Button)dialog.findViewById(R.id.alert_three_button3));

                    button3.setText(responses.getJSONObject(0).getString("value"));
                    button4.setText(responses.getJSONObject(1).getString("value"));
                    button5.setText(responses.getJSONObject(2).getString("value"));

                    // clickable actions for buttons -> add to controller reply queue
                    button3.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(0).getString("id"));
                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(0).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(0).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }
                        }
                    });

                    button4.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(1).getString("id"));

                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(1).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(1).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    button5.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(2).getString("id"));

                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(2).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(2).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    // set status of robot
                    if (status.equals("ok")) {
                        ((ImageView)dialog.findViewById(R.id.alert_three_img)).setImageResource(R.drawable.svg_ok);
                    } else if (status.equals("safe")) {
                        ((ImageView)dialog.findViewById(R.id.alert_three_img)).setImageResource(R.drawable.svg_safe);
                    } else if (status.equals("dangerous")) {
                        ((ImageView)dialog.findViewById(R.id.alert_three_img)).setImageResource(R.drawable.svg_dangerous);
                    } else if (status.equals("help")) {
                        ((ImageView)dialog.findViewById(R.id.alert_three_img)).setImageResource(R.drawable.svg_help);
                    } else if (status.equals("off")) {
                        ((ImageView)dialog.findViewById(R.id.alert_three_img)).setImageResource(R.drawable.svg_off);
                    }

                    // set content of progression
                    ((TextView)dialog.findViewById(R.id.alert_three_content))
                            .setText(Html.fromHtml(progressionElement.getString("content")));

                    // setting clickable action for later and more info
                    ((Button)dialog.findViewById(R.id.alert_three_info)).setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                    RobotLink.class)
                                    .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                    .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));
                            dialog.dismiss();

                            ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                    .format(new Date())
                                    + ",SELECTED,more_info");
                        }
                    });
                    /*
                    ((Button)dialog.findViewById(R.id.alert_three_later)).setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            //get rid of dialog box and don't show again for this robot
                            dialog.dismiss();

                            ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                    + ": Selected later on " + bot.getName()
                                    + " from popup");

                        }
                    });*/

                    break;
                // 4 buttons
                case 4:

                    dialog.setContentView(R.layout.alert_four);
                    Button button6 = ((Button)dialog.findViewById(R.id.alert_four_button1));
                    Button button7 = ((Button)dialog.findViewById(R.id.alert_four_button2));
                    Button button8 = ((Button)dialog.findViewById(R.id.alert_four_button3));
                    Button button9 = ((Button)dialog.findViewById(R.id.alert_four_button4));

                    button6.setText(responses.getJSONObject(0).getString("value"));
                    button7.setText(responses.getJSONObject(1).getString("value"));
                    button8.setText(responses.getJSONObject(2).getString("value"));
                    button9.setText(responses.getJSONObject(3).getString("value"));

                    // clickable actions for buttons -> add to controller reply queue
                    button6.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(0).getString("id"));
                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(0).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(0).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }
                        }
                    });

                    button7.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(1).getString("id"));

                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(1).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(1).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    button8.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(2).getString("id"));

                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(2).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(2).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    button9.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            try {
                                ControllerService.addToReplyQueue(bot.getId(),
                                        progressionElement.getString("msgid"), responses.getJSONObject(3).getString("id"));

                                //end this dialog
                                dialog.dismiss();

                                // start robot link activity with sending message dialog
                                NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                        RobotLink.class)
                                        .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                        //this means this activity was started from popup
                                        .putExtra("EXTRA_CHECKSUM", bot.getStatusHashValue())
                                        .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));

                                /*
                                ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                        + ": Requested " + bot.getName() + " from popup to "
                                        + responses.getJSONObject(3).getString("value"));*/

                                ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                        .format(new Date())
                                        + ",SELECTED,"
                                        + responses.getJSONObject(3).getString("value"));

                            } catch (JSONException ex) {
                                StringWriter stringWriter = new StringWriter();
                                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                ex.printStackTrace(printWriter);
                                Log.e("NotifView.Dialog", stringWriter.toString());
                            }

                        }
                    });

                    // set status of robot
                    if (status.equals("ok")) {
                        ((ImageView)dialog.findViewById(R.id.alert_four_img)).setImageResource(R.drawable.svg_ok);
                    } else if (status.equals("safe")) {
                        ((ImageView)dialog.findViewById(R.id.alert_four_img)).setImageResource(R.drawable.svg_safe);
                    } else if (status.equals("dangerous")) {
                        ((ImageView)dialog.findViewById(R.id.alert_four_img)).setImageResource(R.drawable.svg_dangerous);
                    } else if (status.equals("help")) {
                        ((ImageView)dialog.findViewById(R.id.alert_four_img)).setImageResource(R.drawable.svg_help);
                    } else if (status.equals("off")) {
                        ((ImageView)dialog.findViewById(R.id.alert_four_img)).setImageResource(R.drawable.svg_off);
                    }

                    // set content of progression
                    ((TextView)dialog.findViewById(R.id.alert_four_content))
                            .setText(Html.fromHtml(progressionElement.getString("content")));

                    // setting clickable action for later and more info
                    ((Button)dialog.findViewById(R.id.alert_four_info)).setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            NotificationViewService.this.startActivity(new Intent(NotificationViewService.this,
                                    RobotLink.class)
                                    .putExtra("EXTRA_ROBOT_ID", bot.getId())
                                    .setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));
                            dialog.dismiss();

                            ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                    .format(new Date())
                                    + ",SELECTED,more_info");

                        }
                    });

                    /*
                    ((Button)dialog.findViewById(R.id.alert_four_later)).setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            //get rid of dialog box and don't show again for this robot
                            dialog.dismiss();

                            ControllerService.Log(DateFormat.getTimeInstance().format(new Date())
                                    + ": Selected later on " + bot.getName()
                                    + " from popup");

                        }
                    });
                    */

                    break;
            }

            //add this popup so it won't get displayed again
            //botsThatMessaged.put(bot.getId(), progressionElement.getString("msgid"));

        } catch (JSONException ex) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter, true);
            ex.printStackTrace(printWriter);
            Log.e("NotifView.Dialog", stringWriter.toString());
        }

        // put bot's image in dialog title
        if (bot.getImage() == null) {
            dialog.setFeatureDrawableResource(Window.FEATURE_LEFT_ICON, R.mipmap.ic_launcher);
        } else {
            dialog.setFeatureDrawableResource(Window.FEATURE_LEFT_ICON, bot.getImage());
        }

        //display the dialog
        dialog.show();

        ControllerService.Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",POPUP,"
                + bot.getName());

        // play alert tone
        Ringtone tone = RingtoneManager.getRingtone(getApplicationContext(),
                RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION));
        tone.play();

        // buzz
        ((Vibrator)getSystemService(VIBRATOR_SERVICE)).vibrate(500);




//        // setting layout for dialog
//        LinearLayout dialogLayout = new LinearLayout(this);
//        dialogLayout.setOrientation(LinearLayout.VERTICAL);
//
//        try {
//            // getting content of progression element
//            TextView content = new TextView(this);
//            content.setText(progressionElement.getString("content"));
//            content.setTextSize(18);
//            content.setTextColor(Color.WHITE);
//            // layout rules for content view
//            LinearLayout.LayoutParams contentParams = new LinearLayout.LayoutParams(
//                    LinearLayout.LayoutParams.WRAP_CONTENT,
//                    LinearLayout.LayoutParams.WRAP_CONTENT
//            );
//            contentParams.setMargins(20, 40, 0, 30);
//            content.setLayoutParams(contentParams);
//            dialogLayout.addView(content);
//
//            /**
//             * loop through all responses and make buttons for them
//             */
//            LinearLayout buttonLayout = new LinearLayout(this); // layout that holds buttons
//            buttonLayout.setOrientation(LinearLayout.VERTICAL);
//            // parameters for button container
//            LinearLayout.LayoutParams buttonLayoutParams = new LinearLayout.LayoutParams(
//                    LinearLayout.LayoutParams.WRAP_CONTENT,
//                    LinearLayout.LayoutParams.WRAP_CONTENT
//            );
//            buttonLayoutParams.setMargins(20, 30, 0, 20);
//            buttonLayout.setLayoutParams(buttonLayoutParams);
//            JSONArray responses = progressionElement.getJSONArray("responses");
//            for (int i = 0; i < responses.length(); i++) {
//                // get this element in the response
//                final JSONObject responseElement = responses.getJSONObject(i);
//
//                //parameters for button view
//                LinearLayout.LayoutParams buttonParams = new LinearLayout.LayoutParams(
//                        LinearLayout.LayoutParams.WRAP_CONTENT,
//                        LinearLayout.LayoutParams.WRAP_CONTENT);
//                buttonParams.setMargins(0, 10, 0, 10);
//                Button responseButton = new Button(this);
//                responseButton.setTransformationMethod(null); // remove all caps
//                responseButton.setText(responseElement.getString("value")); // value of response
//                responseButton.setTextSize(20);
//                responseButton.setTextColor(Color.BLACK);
//                responseButton.setBackgroundColor(Color.WHITE);
//                responseButton.setLayoutParams(buttonParams);
//                responseButton.setOnClickListener(new View.OnClickListener() {
//                    @Override
//                    public void onClick(View v) {
//                        try {
//                            ControllerService.addToReplyQueue(bot.getId(),
//                                    progressionElement.getString("msgid"), responseElement.getString("id"));
//
//                            //end this dialog
//                            dialog.cancel();
//                        } catch (JSONException ex) {
//                            StringWriter stringWriter = new StringWriter();
//                            PrintWriter printWriter = new PrintWriter(stringWriter, true);
//                            ex.printStackTrace(printWriter);
//                            Log.e("RobotLink.Progression", stringWriter.toString());
//                        }
//                    }
//                });
//                buttonLayout.addView(responseButton);
//            }
//            // adding buttons to dialog
//            dialogLayout.addView(buttonLayout);
//
//            // add content to dialog
//            dialog.setContentView(dialogLayout);
//
//            // set overall dialog size
//            Window window = dialog.getWindow();
//            window.setLayout(WindowManager.LayoutParams.WRAP_CONTENT, WindowManager.LayoutParams.WRAP_CONTENT);
//
//            //display dialog
//            dialog.show();
//
//        } catch (JSONException ex) {
//            StringWriter stringWriter = new StringWriter();
//            PrintWriter printWriter = new PrintWriter(stringWriter, true);
//            ex.printStackTrace(printWriter);
//            Log.e("NotifView.Dialog", stringWriter.toString());
//        }

    }
}

