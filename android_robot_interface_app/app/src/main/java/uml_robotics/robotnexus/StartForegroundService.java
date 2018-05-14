package uml_robotics.robotnexus;

import android.app.Notification;
import android.app.Service;
import android.content.Intent;
import android.os.IBinder;

/**
 * responsible for putting ControllerService in foreground without displaying a notification
 */

public class StartForegroundService extends Service {
    public StartForegroundService() {}


    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        Notification notification1 = new Notification.Builder(ControllerService.controllerService).build();
        ControllerService.controllerService.startForeground(999, notification1);

        Notification notification2 = new Notification.Builder(this).build();
        this.startForeground(999, notification2);

        stopForeground(true);

        stopSelf();

        return Service.START_NOT_STICKY;
    }

    @Override
    public IBinder onBind(Intent intent) {
      return null;
    }
}
