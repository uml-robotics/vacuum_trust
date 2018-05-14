package edu.uml.cs.balloons;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.util.Log;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;


/**
 * Created by csrobot on 12/30/16.
 */

public class GameElements {
    public static enum GameState {
        READY, // "Waiting for game to begin."
        STANDBY, // "Waiting for game to begin."
        RUNNING, // game go
        PAUSED, // activates "game paused????"
        COMPLETE // activates score screen
    }

    private static final double REG_SPAWN_RATE = 0.08;
    private static final int SKULL_PENALTY = 30;
    private static final String SCORE_FILE_NAME = "score_file.txt";

    private final Bitmap balloonCollisionBmp;

    private GameState gameState = GameState.READY;
    private List<Balloon> balloonList;
    private int score;
    private int popped;
    private int skullsPopped;
    private int time;
    private boolean gameZoneEnabled;
    
    public GameElements(Context context) {
        balloonCollisionBmp =
                BitmapFactory.decodeResource(context.getResources(), R.mipmap.balloon_blue);
        balloonList = new ArrayList<Balloon>();
        score = readScoreFile();
        setTime(MainActivity.getInstance().timeLeftSub.getLastTime());
        setGameZoneDetector(MainActivity.getInstance().gameZoneDetectorSub.getLastDetection());
        String lastStateStr = MainActivity.getInstance().gameStateSub.getLastState();
        if (lastStateStr != null) {
            setGameState(lastStateStr);
        }
    }

    public void update() {
        for (int i = 0; i < balloonList.size(); i++) {
            if (balloonList.get(i).updatePos()) {
                balloonList.remove(i);
                i--;
            }
        }

        double spawnRate = REG_SPAWN_RATE;
        if (balloonList.size() < 20) {
            spawnRate += (20 - balloonList.size()) * 0.01;
        }
        if (spawnRate > Math.random()) {
            balloonList.add(Balloon.getRandomBalloon());
        }
    }

    /**
     * Attempts to pop a balloon at the specified coordinate.
     * @param x the x-coordinate of the pop.
     * @param y the y-coordinate of the pop.
     * @return if it found a balloon to pop.
     */
    public Balloon popBalloonAt(int x, int y) {
        for (int i = balloonList.size() - 1; i >= 0; i--) {
            Balloon balloon = balloonList.get(i);
            int xx = x - balloon.x();
            int yy = y - balloon.y();
            if (xx > 0 && xx < balloonCollisionBmp.getWidth()
                    && yy > 0 && yy < balloonCollisionBmp.getHeight()) {
                int color = balloonCollisionBmp.getPixel(xx, yy);
                if (color != Color.TRANSPARENT) {
                    balloonList.remove(i);
                    popped++;
                    if (balloon.color() != Balloon.BalloonColor.SKULL) {
                        setScore(score + 1);
                    } else if (score >= SKULL_PENALTY){
                        skullsPopped++;
                        setScore(score - SKULL_PENALTY);
                    } else {
                        skullsPopped++;
                        setScore(0);
                    }
                    return balloon;
                }
            }
        }
        return null;
    }

    public void setGameZoneDetector(Boolean isEnabled) {
        gameZoneEnabled = isEnabled;
    }

    public boolean isEnabled() {
        return gameState == GameState.RUNNING && gameZoneEnabled;
    }

    public List<Balloon> getBalloonList() {
        return balloonList;
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
        if (gameState == GameState.READY) {
            reset();
        }
    }

    public void setGameState(String gameStateStr) {
        try {
            setGameState(GameElements.GameState.valueOf(gameStateStr.toUpperCase()));
        } catch (IllegalArgumentException | NullPointerException e) {
            // Occurs when an invalid gameState is provided.
            Log.e("Balloons", e.toString());
        }
    }

    private void setScore(int score) {
        this.score = score;
        MainActivity.getInstance().scorePub.publish(score);
        writeScoreFile();
    }

    private void writeScoreFile() {
        FileOutputStream outputStream;
        try {
            outputStream = MainActivity.getInstance()
                    .getApplicationContext()
                    .openFileOutput(SCORE_FILE_NAME, Context.MODE_PRIVATE);
            outputStream.write((score + "").getBytes());
            outputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private int readScoreFile() {
        String score = "";
        try {
            InputStream inputStream = MainActivity.getInstance()
                    .getApplicationContext()
                    .openFileInput(SCORE_FILE_NAME);

            if ( inputStream != null ) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";
                StringBuilder stringBuilder = new StringBuilder();

                while ( (receiveString = bufferedReader.readLine()) != null ) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                score = stringBuilder.toString();
            }
        }
        catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
            return 0;
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
            return 0;
        }
        return Integer.decode(score);
    }

    private void reset() {
        skullsPopped = 0;
        popped = 0;
        score = 0;
    }

    public void setTime(int seconds) {

        Log.d("Balloons", "time set to: " + seconds);
        time = seconds;
    }

    public GameState gameState() { return gameState; }

    public int skullsPopped() { return skullsPopped; }

    public int popped() { return popped; }

    public int score() { return score; }

    public int time() { return time; }

    public boolean gameZoneEnabled() { return gameZoneEnabled; }

}
