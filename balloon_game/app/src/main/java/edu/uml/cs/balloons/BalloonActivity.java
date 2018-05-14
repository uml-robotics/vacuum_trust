package edu.uml.cs.balloons;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.View;

import java.util.Timer;
import java.util.TimerTask;

public class BalloonActivity extends Activity {

    private static final int FRAME_RATE = 24;
    private static final int SLEEP_TIME = 1000 / FRAME_RATE;
    private static BalloonActivity instance;

    public static BalloonActivity getInstance() {
        return instance;
    }

    private final GameElements gameElements = MainActivity.getInstance().gameElements;
    private final GameView gameView = MainActivity.getInstance().gameView;
    private boolean paused;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        instance = this;
        setContentView(gameView);

        final Handler handler = new Handler();
        Runnable updateRunnable = new Runnable() {
            long time = SystemClock.elapsedRealtime() + 1;
            public void run() {
                gameElements.update();
                gameView.invalidate();
                // enable below code to log fps
//                long frame_time = (SystemClock.elapsedRealtime() - time);
//                if (frame_time != 0)
//                    Log.d("Balloons", "fps: " + (1000 / frame_time));
                time = SystemClock.elapsedRealtime();
                handler.postDelayed(this, SLEEP_TIME);
            }
        };
        handler.post(updateRunnable);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        if (!hasFocus) {
            new Thread() {
                public void run() {
                    try {
                        Thread.sleep(300);
                        if (!paused) {
                            MainActivity.getInstance().statusBarPub.publish(true);
                        }
                    } catch(InterruptedException v) {
                        System.out.println(v);
                    }
                }
            }.start();
        } else if (MainActivity.getInstance().statusBarPub.isStatusBarActive()) {
            MainActivity.getInstance().statusBarPub.publish(false);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        paused = true;
        MainActivity.getInstance().appActivePub.publish(false);
        if (MainActivity.getInstance().statusBarPub.isStatusBarActive()) {
            MainActivity.getInstance().statusBarPub.publish(false);
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        paused = false;
        MainActivity.getInstance().appActivePub.publish(true);
    }

    public GameElements gameElements() {
        return gameElements;
    }

    public GameView gameView() {
        return gameView;
    }

    @Override
    public void onBackPressed() {
    }
}
