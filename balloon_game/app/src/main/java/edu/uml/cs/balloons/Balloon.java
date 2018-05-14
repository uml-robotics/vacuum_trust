package edu.uml.cs.balloons;

import android.content.res.Resources;

/**
 * Created by csrobot on 12/28/16.
 */

public class Balloon {
    public static enum BalloonColor {
        BLUE,
        GREEN,
        PINK,
        RED,
        YELLOW,
        SKULL;

        public static BalloonColor getRandomColor() {
            return BalloonColor.values()[(int)(Math.random() * (BalloonColor.values().length))];
        }
    }

    private static int MIN_SPEED = 4;
    private static int SPEED_DEVIATION = 10;
    private static int FAST_SPEED_INC = 16;
    private static int HEIGHT = 274;

    private int x;
    private int y = 0;
    private int speed;
    private BalloonColor color;

    public Balloon(int x, int y, BalloonColor color, int speed) {
        this.x = x;
        this.y = y;
        this.color = color;
        this.speed = speed;
    }

    /**
     * Moves the balloon.
     * @return true if balloon has moved off the screen.
     */
    public boolean updatePos() {
        y -= speed;
        return y < -HEIGHT;
    }

    public int x() {
        return x;
    }

    public int y() {
        return y;
    }

    public BalloonColor color() {
        return color;
    }

    public static Balloon getRandomBalloon() {
        int randomX = (int) ((Math.random() * .96 - .09)
                * Resources.getSystem().getDisplayMetrics().widthPixels);
        int randomY = Resources.getSystem().getDisplayMetrics().heightPixels;
        BalloonColor randomColor = BalloonColor.getRandomColor();
        int randomSpeed = MIN_SPEED + (int) (Math.random() * SPEED_DEVIATION);
        if (Math.random() < 0.3) {
            randomSpeed += FAST_SPEED_INC;
        }
        return new Balloon(randomX, randomY, randomColor, randomSpeed);
    }
}
