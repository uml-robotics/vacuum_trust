package edu.uml.cs.balloons;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.media.AudioAttributes;
import android.media.AudioManager;
import android.media.SoundPool;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewTreeObserver;

import java.util.List;
import java.util.Locale;

/**
 * Created by csrobot on 12/30/16.
 */

public class GameView extends View implements View.OnTouchListener {

    private final int BACKGROUND_COLOR = Color.parseColor("#303070");
    private final int GOLD_TEXT_COLOR = Color.parseColor("#ffd700");
    private final int TEXT_PADDING = 50;
    private final int SCOREBOARD_MARGIN = 130;

    private final GameElements gameElements;
    private Paint disabledPaint;
    private Paint fontPaint;
    private Paint scoreboardFontPaint;
    private Paint paint;

    public final Bitmap blueBmp =
            BitmapFactory.decodeResource(getResources(), R.mipmap.balloon_blue);
    public final Bitmap greenBmp =
            BitmapFactory.decodeResource(getResources(), R.mipmap.balloon_green);
    public final Bitmap pinkBmp =
            BitmapFactory.decodeResource(getResources(), R.mipmap.balloon_pink);
    public final Bitmap redBmp =
            BitmapFactory.decodeResource(getResources(), R.mipmap.balloon_red);
    public final Bitmap yellowBmp =
            BitmapFactory.decodeResource(getResources(), R.mipmap.balloon_yellow);
    public final Bitmap skullBmp =
            BitmapFactory.decodeResource(getResources(), R.mipmap.balloon_bad);
    public Bitmap gamePreparingBmp;
    public Bitmap gameDisabledBmp;
    public Bitmap gamePausedBmp;
    public Bitmap scoreboardBmp;

    public final SoundPool soundPool;
    public final int popSoundId;
    public final int badPopSoundId;

    public GameView(Context context, GameElements gameElements) {
        super(context);
        this.gameElements = gameElements;
        setDrawingCacheEnabled(true);
        setOnTouchListener(this);

        soundPool = new SoundPool(5, AudioManager.STREAM_MUSIC, 0);
        popSoundId = soundPool.load(context, R.raw.pop, 1);
        badPopSoundId = soundPool.load(context, R.raw.burst, 1);

        setupPaints();

        final View self = this;
        getViewTreeObserver().addOnGlobalLayoutListener(
                new ViewTreeObserver.OnGlobalLayoutListener() {
                    @Override
                    public void onGlobalLayout() {
                        self.getViewTreeObserver().removeGlobalOnLayoutListener(this);
                        constructScaledBmps();
                    }
                });
    }

    private void setupPaints() {
        paint = new Paint();
        paint.setColor(BACKGROUND_COLOR);
        fontPaint = new Paint();
        fontPaint.setColor(GOLD_TEXT_COLOR);
        fontPaint.setTextSize(80);
        scoreboardFontPaint = new Paint();
        scoreboardFontPaint.setColor(Color.WHITE);
        scoreboardFontPaint.setTextSize(80);
        scoreboardFontPaint.setTypeface(Typeface.DEFAULT_BOLD);
        disabledPaint = new Paint();
        disabledPaint.setColor(Color.BLACK);
        disabledPaint.setAlpha(120);
    }

    private void constructScaledBmps() {
        gamePreparingBmp = createScaledBmp(R.mipmap.game_preparing);
        gameDisabledBmp = createScaledBmp(R.mipmap.game_disabled);
        gamePausedBmp = createScaledBmp(R.mipmap.game_paused);
        scoreboardBmp = createScaledBmp(R.mipmap.scoreboard);
    }

    private Bitmap createScaledBmp(int resId) {
        Bitmap unscaledBmp =
                BitmapFactory.decodeResource(getResources(), resId);
        double scale = ((double) this.getWidth()) / unscaledBmp.getWidth();
        return Bitmap.createScaledBitmap(
                unscaledBmp,
                this.getWidth(),
                (int) (unscaledBmp.getHeight() * scale), false);
    }

    @Override
    public void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        canvas.drawPaint(paint);
        for(Balloon balloon : gameElements.getBalloonList()) {
            Bitmap bitmap = getBalloonBitmap(balloon.color());
            canvas.drawBitmap(bitmap, balloon.x(), balloon.y(), paint);
        }
        if (!gameElements.isEnabled()) {
            canvas.drawPaint(disabledPaint);
        }
        switch (gameElements.gameState()) {
            case READY:
            case STANDBY:
                drawImageCenter(canvas, gamePreparingBmp);
                break;
            case RUNNING:
                if (!gameElements.gameZoneEnabled()) {
                    drawImageCenter(canvas, gameDisabledBmp);
                }
                break;
            case PAUSED:
                drawImageCenter(canvas, gamePausedBmp);
                break;
            case COMPLETE: {
                drawImageCenter(canvas, scoreboardBmp);
                String scoreTxt = "SCORE: " + gameElements.score();
                canvas.drawText(scoreTxt,
                        canvas.getWidth() / 2 - scoreboardFontPaint.measureText(scoreTxt) / 2,
                        canvas.getHeight() / 2,
                        scoreboardFontPaint);

                String balloonsTxt = "POPPED: " + gameElements.popped();
                canvas.drawText(balloonsTxt,
                        canvas.getWidth() / 2 - scoreboardFontPaint.measureText(balloonsTxt) / 2,
                        canvas.getHeight() / 2 + SCOREBOARD_MARGIN,
                        scoreboardFontPaint);

                String skullsTxt = "SKULLS POPPED: " + gameElements.skullsPopped();
                canvas.drawText(skullsTxt,
                        canvas.getWidth() / 2 - scoreboardFontPaint.measureText(skullsTxt) / 2,
                        canvas.getHeight() / 2 + 2 * SCOREBOARD_MARGIN,
                        scoreboardFontPaint);
                break;
            }
        }
        if (gameElements.gameState() != GameElements.GameState.COMPLETE) {
            drawTimeAndScore(canvas);
        }
    }

    private void drawImageCenter(Canvas canvas, Bitmap bitmap) {
        int top = canvas.getHeight() / 2 - bitmap.getScaledHeight(canvas) / 2;
        int left = canvas.getWidth() / 2 - bitmap.getScaledWidth(canvas) / 2;
        canvas.drawBitmap(bitmap, left, top, paint);
    }

    private Rect bounds = new Rect();
    private void drawTimeAndScore(Canvas canvas) {
        int sec = gameElements.time();
        String timeTxt = "Time: " + String.format(Locale.ENGLISH, "%02d:%02d", sec/60, sec%60);
        fontPaint.getTextBounds(timeTxt, 0, timeTxt.length(), bounds);
        canvas.drawText(timeTxt,
                canvas.getWidth() - TEXT_PADDING - bounds.width(),
                TEXT_PADDING + bounds.height(),
                fontPaint);
        canvas.drawText("Score: " + gameElements.score(),
                TEXT_PADDING,
                TEXT_PADDING + bounds.height(),
                fontPaint);
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        if (gameElements.isEnabled() && event.getAction() == MotionEvent.ACTION_DOWN) {
            Balloon popped = gameElements.popBalloonAt((int)event.getX(), (int)event.getY());
            if (popped != null) {
                if (popped.color() != Balloon.BalloonColor.SKULL) {
                    soundPool.play(popSoundId, 1, 1, 0, 0, 1);
                } else {
                    soundPool.play(badPopSoundId, 0.3f, 0.3f, 0, 0, 1);
                }
            }
        }
        return false;
    }

    public Bitmap getBalloonBitmap(Balloon.BalloonColor balloonColor) {
        switch (balloonColor) {
            case BLUE:
                return blueBmp;
            case GREEN:
                return greenBmp;
            case PINK:
                return pinkBmp;
            case RED:
                return redBmp;
            case YELLOW:
                return yellowBmp;
            case SKULL:
                return skullBmp;
            default:
                return null;
        }
    }

    public void setGameTime(int seconds) {
        gameElements.setTime(seconds);
    }

    public void setGameState(String gameState) {
        gameElements.setGameState(gameState);
    }

    public GameElements getGameElements() {
        return gameElements;
    }
}
