package uml_robotics.robotnexus;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Handler;
import android.provider.Settings;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import static java.lang.Thread.sleep;

/*
 * DON'T do heavy work in CALLBACKS (~3 threads in pool )
 *
 */

/*
 * * Dec 22, 2016
 * * created project
 * created controller and RobotSelector view
 * created Robot object that the model will contain an array list of
 * finished onstartcommand stuff in controllerservice
 *
 *
 * * Dec 28, 2016
 * * changed theme of app
 *
 *
 * * Dec 30, 2016
 * * worked on new dialog protocol
 *
 */

public class MainActivity extends Activity {
    private Intent controllerIntent;
    private boolean startedRobotSelector = false;
    private int shutdownClicks = 0; //5 clicks allows for app to be destroyed on exit

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.loading_screen);

        // source: http://stackoverflow.com/questions/7569937/unable-to-add-window-android-view-viewrootw44da9bc0-permission-denied-for-t#answer-34061521
        /** check if we already  have permission to draw over other apps */
        if (!Settings.canDrawOverlays(this)) {

            /** if not construct intent to request permission */
            Intent intent = new Intent(Settings.ACTION_MANAGE_OVERLAY_PERMISSION,
                    Uri.parse("package:" + getPackageName()));
            /** request permission via start activity for result */
            startActivityForResult(intent, -1010101);

        } else {

            ControllerService controllerService = new ControllerService();

            if (!(isServiceRunning(controllerService.getClass()))) {
                //boot up controller
                controllerIntent = new Intent(this, ControllerService.class);
                this.startService(controllerIntent);
                Log.i("MAIN.onCreate()", "Started Controller");
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (!startedRobotSelector) {
            startedRobotSelector = true;
            new Handler(getMainLooper()).postDelayed(new Runnable() {
                @Override
                public void run() {
                    // starting the *real main activity* which will be a navigation screen for all robots in the area
                    MainActivity.this.startActivity(new Intent(MainActivity.this, RobotSelector.class));
                    Log.i("MAIN.onResume()", "Transitioning to RobotSelector activity");
                }
            }, 3000);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (shutdownClicks >= 5) {
            this.stopService(ControllerService.controllerServiceIntent);
            Log.i("MAIN.onDestroy()", "Stopped Controller");
        }

    }

    // helper method for determining if a service (owned by robot nexus) is running
    private boolean isServiceRunning(Class serviceClass) {
        ActivityManager manager = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);
        for (ActivityManager.RunningServiceInfo service : manager.getRunningServices(Integer.MAX_VALUE)) {
            if (serviceClass.getName().equals(service.service.getClassName())) {
                return true;
            }
        }
        return false;
    }

    // source: http://stackoverflow.com/questions/7569937/unable-to-add-window-android-view-viewrootw44da9bc0-permission-denied-for-t#answer-34061521
    @Override
    protected void onActivityResult(int requestCode, int resultCode,  Intent data) {
        /** check if received result code
         is equal our requested code for draw permission  */
        if (requestCode == -1010101) {
            /** if so check once again if we have permission */
            if (Settings.canDrawOverlays(this)) {
                // continue here - permission was granted

                ControllerService controllerService = new ControllerService();
                if (!(isServiceRunning(controllerService.getClass()))) {
                    //boot up controller
                    controllerIntent = new Intent(this, ControllerService.class);
                    this.startService(controllerIntent);
                    Log.i("MAIN.onCreate()", "Started Controller");
                }

                // starting the *real main activity* which will be a navigation screen for all robots in the area
                this.startActivity(new Intent(MainActivity.this, RobotSelector.class));
                Log.i("MAIN.onCreate()", "Transitioning to RobotSelector activity");
            }
        }
    }

    // hidden button that needs to be clicked 10 times to allow for successful shutdown of app
    public void shutdownButtonClick(View v) {
        Log.i("MAIN.shutdownClick", "CLICK");
        shutdownClicks++;
        if (shutdownClicks == 5) {
            Toast.makeText(MainActivity.this, "Exit app to shutdown", Toast.LENGTH_LONG).show();
        }
    }
}
