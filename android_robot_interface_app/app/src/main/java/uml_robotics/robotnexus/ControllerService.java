package uml_robotics.robotnexus;

import android.app.NotificationManager;
import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.util.Base64;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Array;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.zip.CRC32;

import static java.lang.Thread.sleep;

public class ControllerService extends Service {
    // accessed by StartForegroundService to get this service's instance
    protected static ControllerService controllerService;
    protected static Intent controllerServiceIntent; //accessed by main activity to shutdown app
    // action name to extract the jpeg from strJSON
    private final String DESERIALIZE_JPEG = "uml_robotics.controller.deserialize_jpeg";
    private final String UPDATE_COMPLETE = "uml_robotics.controller.update_complete";
    private ServiceLooper serviceLooper; //looper to maintain a service thread
    private Handler serviceHandler; // handler for posting to looper
    private static ArrayList<Robot> theModel; // the model of the system -> only manipulate through its methods
    private static ReentrantLock theModelLock; // keeps mutual exclusion of model tampering
    private Intent notifViewIntent; // intent used to start notification ui service
    private ArrayList<Robot> model; // controller's copy of the model
    private ReentrantLock modelLock; // mutex for tampering with copy of the model
    private Boolean btInitOff = false; //used for checking initial state of user's bluetooth
    private BluetoothAdapter btAdapter;  //Adapter used for most bluetoothy stuff
    private BluetoothLeScanner leScanner; // used for scanning
    private ScanCallback scanCallback;  // callback for leScannner scans
    // callback that triggers when le devices are found by startLeScan()
   // private BluetoothAdapter.LeScanCallback leCallback;
    // callback for gattserver events
    private BluetoothGattCallback btGattCallback;
    private boolean isScanning = false; //for tracking if app is scanning or not
    private UUID uuidOfInterest; //UUID that specifies this is a robot
    private HashMap<String, String> supportedServices;//known services
    private HashMap<String, String> supportedCharas; //known characteristics
    //Holds bluetooth devices we don't care about
    private List<BluetoothDevice> rejectedDeviceList = null;
    // holds bluetooth devices identified as robots
    private HashMap<BluetoothDevice, Integer> robotsAsBTDevices;
    private HashMap<Integer, String> packetsFound = null; // map multi packet reads
    // used in makeRobot() for reading characteristics and
    // stopping from finishing until descriptor has been written (because image sends after read request)
    private BlockingQueue<Integer> makeRobotBlock = null;
    // lock for sequencing statusReview and readNotifications
    private ReentrantLock transferLock;
    // queue for scan callbacks
    private ArrayList<ScanCallbackPackage> scanCallbackPackages;
    private ReentrantLock scanCallbackPackagesLock = null; // lock for accessing package queue
    private HandleScanCallbacks handleScanCallbacks; // thread responsible for handling scan callbacks
    // boolean for stating if we're connected to a robot or not
    private boolean isConnected = false;
    private BluetoothGatt currConnectedDevice = null; //current device we are connected to
    // used for determining when server has stopped notifying
    private TransferStatusReview statusReview = null;
    // time stamp updated on each onCharacteristicChanged callback
    private long notifyTimeStamp;
    private String strJSON = null; // contains JSON string from server
    // boolean to let others know if TransferStatusReview is on or not
    private boolean statusReviewOff = true;
    private boolean awaitingMissedPackets = false; //used to know if we should expect a missing packet
    private BroadcastReceiver receiver; // listener for controller
    private ArrayList<byte[]> notificationQueue = null; // queue for holding notification values
    private ReentrantLock notificationQueueLock = null; // lock for accessing queue
    private ReadNotifications readNotifications; // thread responsible for reading notifications
    // used to ensure all robots in vicinity get updated once
    private RobotUpdateClock robotUpdateClock;
    private BlockingQueue<Integer> characteristicWriteBlock = null;
    // Controller only uses this to ensure notifications are removed on destroy
    private NotificationManager notifManager;
    // queue for replies waiting to be sent back to robot
    private static ArrayList<ReplyPackage> replyQueue = null;
    private static ReentrantLock replyQueueLock = null; // lock for accessing reply queue
    // boolean that holds the visible value in the advertisement -> is global because no way to pass
    // to connect and discover callbacks
    private boolean isCurrRobotVisible = true;
    //holds the date and time of when the app was started
    private static String dateAndTimeOfAppStart = null;
    //holds current batch of packets being sent to server
    private HashMap<Integer, byte[]> outgoingPackets;

    /*
     * characteristics/values of our currently connected robot
     */
    private int totalNumOfPackets = -1;
    private BluetoothGattCharacteristic missingPacketWrite = null;
    private BluetoothGattCharacteristic totalNumOfPacketsWrite = null;
    private BluetoothGattCharacteristic packetWrite = null;
    /*
     * end fields for currently connected robot
     */

    public ControllerService() {}

    @Override
    public void onCreate() {
        final String TAG = "Controller.onCreate()";
        Log.i(TAG, "Service created");

        // make the model and its lock
        theModel = new ArrayList<Robot>();
        theModelLock = new ReentrantLock();

        // create the service thread
        serviceLooper = new ServiceLooper();
        serviceLooper.start();

        // set this instance
        controllerService = this;

        // getting date and time
        //SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss");
        //dateAndTimeOfAppStart = DateFormat.getDateTimeInstance().format(new Date());
        //dateAndTimeOfAppStart = format.format(new Date());
        dateAndTimeOfAppStart = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss").format(new Date());

        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",INITIALIZED,robotlinkapp");

        // sleeping for one second to give looper a chance to make handler
        try {
            sleep(1000);
        } catch (Exception ex) {
            Log.i(TAG, "Failed sleep to give service thread time.");
        }

        // post to service message queue
        serviceHandler.post(new Runnable() {
            @Override
            public void run() {

                // enable the controller as a foreground service without notification
                startService(new Intent(ControllerService.this, StartForegroundService.class));

                // boot up notification service
                notifViewIntent = new Intent(ControllerService.this, NotificationViewService.class);
                ControllerService.this.startService(notifViewIntent);
                Log.i("Controller.onCreate()", "Started NotificationViewService");

                // get notification manager
                notifManager = (NotificationManager)getSystemService(Context.NOTIFICATION_SERVICE);

                //Getting app's btAdapter
                //BluetoothManager bluetoothManager = (BluetoothManager)getSystemService(BLUETOOTH_SERVICE);
                btAdapter = BluetoothAdapter.getDefaultAdapter();

                //Turning on bluetooth if it is currently off
                if (!btAdapter.isEnabled()) {

                    btAdapter.enable();
                    // user did not have bt on
                    btInitOff = true;

                }

                //setting our robot uuid (this is sent in the advertisement)
                //uuidOfInterest = UUID.fromString("0000ec00-0000-1000-8000-00805f9b34fb");
                //uuidOfInterest = UUID.fromString("00001800-30de-4630-9b59-27228d45bf11");
                uuidOfInterest = UUID.fromString("11bf458d-2227-599b-3046-de3000180000");

                //Populating supportedServices and supportedCharas maps
                supportedServices = new HashMap<String, String>();
                //supportedServices.put("00001800-0000-1000-8000-00805f9b34fb", "Generic Access");
                //supportedServices.put("00001801-0000-1000-8000-00805f9b34fb", "Generic Attribute");
                supportedServices.put("00001800-30de-4630-9b59-27228d45bf11", "Image Receive");
                supportedServices.put("00001801-30de-4630-9b59-27228d45bf11", "Image Send");

                supportedCharas = new HashMap<String, String>();
                //supportedCharas.put("00002a00-0000-1000-8000-00805f9b34fb", "Device Name");
                //supportedCharas.put("00002a01-0000-1000-8000-00805f9b34fb", "Appearance");
                //supportedCharas.put("00002a05-0000-1000-8000-00805f9b34fb", "Service Changed");
                supportedCharas.put("00002a10-30de-4630-9b59-27228d45bf11", "Packet Read");
                supportedCharas.put("00002a11-30de-4630-9b59-27228d45bf11", "Missing Packet Write");
                supportedCharas.put("00002a12-30de-4630-9b59-27228d45bf11", "Packet Write");
                supportedCharas.put("00002a13-30de-4630-9b59-27228d45bf11", "Missing Packet Read");
                supportedCharas.put("00002a14-30de-4630-9b59-27228d45bf11", "Total Number of Packets");

                //List of rejected bluetooth devices
                rejectedDeviceList = new ArrayList<BluetoothDevice>();

                // non-connected btDevices(that are robots) with their average rssis
                robotsAsBTDevices = new HashMap<BluetoothDevice, Integer>();

                // initializing packetFound map for multi packet reads.
                packetsFound = new HashMap<>();

                // creating a lock for our transfer process
                transferLock = new ReentrantLock();

                //instantiating block for notify completion wait
                makeRobotBlock = new ArrayBlockingQueue<>(1);

                //block for writing to server
                characteristicWriteBlock = new ArrayBlockingQueue<Integer>(1);

                //creating lock for when controller is accessing its own copy of the model
                modelLock = new ReentrantLock();

                // queue of notifications
                notificationQueue = new ArrayList<byte[]>();

                // Lock for notification queue
                notificationQueueLock = new ReentrantLock();

                // clock to ensure all robots get updated
                robotUpdateClock = new RobotUpdateClock();

                // create job queue for scan callbacks
                scanCallbackPackages = new ArrayList<ScanCallbackPackage>();

                //lock for accessing scanCallbackPackages
                scanCallbackPackagesLock = new ReentrantLock();

                // start up handler thread for scan callbacks
                handleScanCallbacks = new HandleScanCallbacks();
                handleScanCallbacks.start();

                //queue for replies back to server
                replyQueue = new ArrayList<ReplyPackage>();

                // lock for the reply queue -> Controller and RobotLink both access
                replyQueueLock = new ReentrantLock();


                // used if server misses packets we send to it
                outgoingPackets = new HashMap<Integer, byte[]>();


                //implementing callback startScan()
                scanCallback = new ScanCallback() {
                    @Override
                    public void onScanResult(int callbackType, ScanResult result) {
                        scanCallbackPackagesLock.lock();
                        scanCallbackPackages.add(new ScanCallbackPackage(result.getDevice(),
                                result.getRssi(), result.getScanRecord().getBytes()));
                        scanCallbackPackagesLock.unlock();
                    }
                };

                // implementing callback for startLeScan()
                /*
                leCallback = new BluetoothAdapter.LeScanCallback() {

                    @Override
                    public void onLeScan(final BluetoothDevice device, final int rssi,
                                                      final byte[] scanRecord) {

                        //Log.i("leCallback", "Start");

                        scanCallbackPackagesLock.lock();
                        //enqueue job for handleScanCallback to work with
                        scanCallbackPackages.add(new ScanCallbackPackage(device, rssi, scanRecord));
                        scanCallbackPackagesLock.unlock();
                        // get on service thread
                        //serviceHandler.post(new Runnable() {
                            //@Override
                            //public void run() {
                               // handleScanCallback(device, rssi, scanRecord);
                           // }
                        //});
                    }
                }; */

                btGattCallback = new BluetoothGattCallback() {
                    @Override
                    public void onConnectionStateChange(final BluetoothGatt gatt, final int status, final int newState) {

                        Log.i("onConnectionStateChange", "Status: " + status);
                        Log.i("onConnectionStateChange", "newState: " + newState);
                        serviceHandler.post(new Runnable() {
                            @Override
                            public void run() {

                                // if this disconnect is 133 -> have to close down BT in order
                                // to close connection down
                                if (status == 133) {
                                    Log.e("onDisconnect", "closing connection and restarting bluetooth");
                                    Log.e("onDisconnect", "Status: " + status);
                                    Log.e("onDisconnect", "newState: " + newState);
                                    gatt.disconnect();
                                    gatt.close();
                                    currConnectedDevice = null;
                                    isConnected = false;
                                    // make sure these blocking queues are cleared if we
                                    // disconnected abruptly
                                    makeRobotBlock.clear();
                                    characteristicWriteBlock.clear();
                                    // safety-net
                                    if (btAdapter != null) {
                                        // resetting BT
                                        btAdapter.disable();
                                        while (btAdapter.isEnabled()) {
                                            try {
                                                Log.i("onDisconnect", "Putting service thread to sleep");
                                                sleep(500);
                                            } catch (Exception ex) {
                                            }
                                        }
                                        //reacquire bt adapter
                                        //btAdapter = BluetoothAdapter.getDefaultAdapter();
                                        btAdapter.enable();
                                        robotsAsBTDevices.clear();
                                        onStartCommandSeparateThread();
                                    }
                                    return;
                                }

                                if (newState == BluetoothProfile.STATE_CONNECTED) {
                                    if (status == BluetoothGatt.GATT_SUCCESS) {

                                        Log.i("onConnect", "Connected to " + (gatt.getDevice()).getName());
                                        currConnectedDevice = gatt;

                                        if (currConnectedDevice.discoverServices()) {

                                        } else {
                                            Log.e("onConnect", "Failed service discovery");
                                            gatt.disconnect();
                                            gatt.close();
                                            currConnectedDevice = null;
                                            isConnected = false;
                                            // safety-net
                                            if (btAdapter != null) {
                                                // resetting BT
                                                btAdapter.disable();
                                                while (btAdapter.isEnabled()) {
                                                    try {
                                                        Log.i("onConnect", "Putting service thread to sleep");
                                                        sleep(500);
                                                    } catch (Exception ex) {
                                                    }
                                                }
                                                btAdapter.enable();
                                                onStartCommandSeparateThread();
                                                robotUpdateClock.startTimer();
                                            }
                                            return;
                                        }
                                    } else {
                                        Log.i("onConnect", "Failed connection with status = " + status);
                                    }
                                }

                                if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                                    //testTotal = 0;
                                    //Log.i("onDisconnect", "Disconnected with status " + status);
                                    Log.i("onDisconnect", "Disconnected from " + (gatt.getDevice()).getName());
                                    try {
                                        currConnectedDevice.close();
                                        currConnectedDevice = null;
                                        isConnected = false;

                                        //tell RobotInfo to end
                                        //sendBroadcast(new Intent().setAction(FINISH));

                                        // end transfer review if we disconnected during transfer process
                                        if (!statusReviewOff) {
                                            statusReview.close();
                                            statusReview = null;
                                        }

                                        // end notification read (safety if we disconnect during transfer)
                                        if (readNotifications != null) {
                                            readNotifications.close();
                                        }

                                        //**DEMO** end tracking of robot proximity
                                        //tracker.close();
                                        //tracker = null;

                                        //DeviceUtilities.clear();
                                        //arAdapter.clear();
                                        //robot.clean();
                                        //DeviceUtilities.robot = null;
                                        strJSON = null;

                                        //safety-net
                                        //if (notifManager != null) {
                                        //notifManager.cancelAll();
                                        //}

                                        // make sure these blocking queues are cleared if we
                                        // disconnected abruptly
                                        makeRobotBlock.clear();
                                        characteristicWriteBlock.clear();

                                        // safety-net
                                        if (btAdapter != null) {
                                            leScanner.startScan(null,
                                                    new ScanSettings.Builder()
                                                            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                                                            //.setReportDelay(0)
                                                            //.setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
                                                            //.setMatchMode(ScanSettings.MATCH_MODE_AGGRESSIVE)
                                                            //.setNumOfMatches(ScanSettings.MATCH_NUM_MAX_ADVERTISEMENT)
                                                            .build(),
                                                    scanCallback);
                                            //starting scan on service thread
                                            /*
                                            while (!(btAdapter.startLeScan(leCallback))) {
                                                try {
                                                    Log.i("onDisconnect", "Failed to start scan. Re-attempting to start scan");
                                                    sleep(500);
                                                } catch (InterruptedException ex) {
                                                    Log.e("onDisconnect", "Service thread failed to sleep.");
                                                }
                                            }*/
                                            isScanning = true;
                                            robotUpdateClock.startTimer();
                                        }
                                    } catch (Exception ex) {
                                        StringWriter stringWriter = new StringWriter();
                                        PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                        ex.printStackTrace(printWriter);
                                        Log.e("Controller.onDisconnect", stringWriter.toString());
                                    }
                                }
                            }
                        });
                    }

                    @Override
                    public void onServicesDiscovered(final BluetoothGatt gatt, final int status) {
                        Log.i("onServicesDiscovered", "Status: " + status);

                        serviceHandler.post(new Runnable() {
                            @Override
                            public void run() {

                                if (status == BluetoothGatt.GATT_SUCCESS) {

                                    // have to run in separate thread because makeRobot is a blocking call
                                    new Thread(new Runnable() {
                                        @Override
                                        public void run() {

                                            makeRobot();
                                            /*
                                            DeviceUtilities.robot = robot;
                                            if (MainActivity.isOnMain) {
                                                //main thread
                                                new Handler(Looper.getMainLooper()).post(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        //start a new activity to handle conveying info about robot to user
                                                        startActivity(new Intent(LinkService.this, RobotInfo.class).setFlags(Intent.FLAG_ACTIVITY_NEW_TASK));
                                                    }
                                                });

                                            } else {

                                                //sending notification that also directs to RobotInfo activity
                                                Log.i("onServicesDiscovered()", "Sending notification");
                                                pullNotificationOn();
                                            }
                                            // **DEMO** starting proximity tracking
                                            //tracker = new TrackRobotProximity();
                                            //tracker.start();
                                            */
                                        }
                                    }).start();
                                }

                            }
                        });
                    }


                    @Override
                    public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {


                        if (status == BluetoothGatt.GATT_SUCCESS) {
                            //finished reading so add our characteristic to the blocking queue
                            makeRobotBlock.add(1);
                            //Log.i("onCharacteristicRead", "finished reading");
                            //Log.i("onCharacteristicRead",
                                    //java.nio.ByteBuffer.wrap(characteristic.getValue()).getInt() + "");
                            //Log.i("onCharacteristicRead", supportedCharas.get(characteristic.getUuid().toString()));
                            totalNumOfPackets = java.nio.ByteBuffer.wrap(characteristic.getValue()).getInt();

                            //if this happens then get out!
                            if (totalNumOfPackets < 1) {
                                currConnectedDevice.disconnect();
                            }

                            // initialize map with correct number of packets wanted
                            if (totalNumOfPackets >= 128) {
                                for (int i = 0; i < 128; i++) {
                                    packetsFound.put(i, "");
                                }
                            } else {
                                for (int i = 0; i < totalNumOfPackets; i++) {
                                    packetsFound.put(i, "");
                                }
                            }
                        } else {
                            Log.e("onCharacteristicRead", "Reading failed: " + characteristic.getUuid().toString());
                            makeRobotBlock.add(1);
                            currConnectedDevice.disconnect();
                        }
                    }

                    @Override
                    public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status) {
                        if (status == BluetoothGatt.GATT_SUCCESS) {
                            // **DEMO** update rssi list for taking averages
                            //if (tracker != null) {
                                //tracker.updateRssiList(rssi);
                           // }
                        }
                    }


                    @Override
                    public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor,
                                                  int status) {
                        if (status == BluetoothGatt.GATT_SUCCESS) {
                            Log.i("onDescriptorWrite", "Successfully Written");
                            // blocking queue add for packet read characteristic
                            makeRobotBlock.add(1);
                        } else {
                            Log.e("onDescriptorWrite", "Write failed");
                            currConnectedDevice.disconnect();
                            makeRobotBlock.add(1);
                        }
                    }


                    @Override
                    public void onCharacteristicChanged(BluetoothGatt gatt,
                                                        final BluetoothGattCharacteristic characteristic) {
                        Log.i("onCharaChange", "Successfully notified: " + characteristic.getUuid().toString());


                            // for reasons unknown we cannot post onto service thread
                            // (or even another work thread) lest multiple characteristics
                            // from the same notification appear

                            //serviceHandler.post(new Runnable() {
                            //@Override
                            //public void run() {

                            // handle new value in characteristic
                            // this characteristic sends an update
                            if (characteristic.
                                    getUuid().toString().equals("00002a10-30de-4630-9b59-27228d45bf11")) {
                                notificationQueueLock.lock();
                                // this characteristic is sending update - add to queue
                                notificationQueue.add(characteristic.getValue());
                                notificationQueueLock.unlock();

                            } else if (characteristic.
                                    getUuid().toString().equals("00002a13-30de-4630-9b59-27228d45bf11")) {
                                // this characteristic is saying if they missed a packet or not
                                Log.i("onCharaChange", "Indicate: "
                                        + Integer.toBinaryString((characteristic.getValue()[0] & 0b11111111) + 256).substring(1)
                                + " : Length: " + characteristic.getValue().length);

                                try {
                                    if (characteristic.getValue()[0] != 0) {
                                        Log.i("onCharaChange", "Something weird happened -> resend");
                                        handleMissingPacketRequest(characteristic.getValue());
                                    } else {
                                        outgoingPackets.clear();
                                        makeRobotBlock.add(1); // proceed
                                    }
                                } catch (Exception ex){
                                    StringWriter stringWriter = new StringWriter();
                                    PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                    ex.printStackTrace(printWriter);
                                    Log.e("onCharaChange", stringWriter.toString());
                                    currConnectedDevice.disconnect();
                                    makeRobotBlock.clear();
                                }
                            }

                    }


                    @Override
                    public void onCharacteristicWrite(BluetoothGatt gatt,
                                                      BluetoothGattCharacteristic characteristic,
                                                      int status) {
                        if (status == BluetoothGatt.GATT_SUCCESS) {
                            Log.i("OnCharaWrite", "Successfully written: " + characteristic.getUuid());
                            characteristicWriteBlock.add(1);
                        } else {
                            Log.i("OnCharaWrite", "Unsuccessful write: " + characteristic.getUuid());
                            characteristicWriteBlock.add(1);
                        }
                    }
                };

                receiver = new BroadcastReceiver() {

                    @Override
                    public void onReceive(final Context context, final Intent intent) {
                        serviceHandler.post(new Runnable() {
                            @Override
                            public void run() {

                                String action = intent.getAction();

                        /*
                        if (DISMISS.equals(action)) {
                            // User clicks notification dismiss or green button in robotinfo

                            Log.i("DISMISS.receiver", "Inside dismiss block");
                            dismissedList.add(new DismissedRobot(DeviceUtilities.robot.getGattServer().getDevice()));
                            DeviceUtilities.robot.getGattServer().disconnect();

                        } else */
                                /*
                                if (DESERIALIZE_JPEG.equals(action)) {
                                    // image transfer has been completed

                                    try {

                                        JSONObject jsonJPEG = new JSONObject(strJSON);
                                        Log.i("JPEG.RECEIVER", jsonJPEG.getString("img"));
                                        FileOutputStream output = context.openFileOutput("img.jpg", Context.MODE_PRIVATE);
                                        output.write(Base64.decode(jsonJPEG.getString("img"), Base64.DEFAULT));
                                        output.flush();
                                        output.close();
                                        Bitmap image = BitmapFactory.decodeFile(context.getFilesDir().getAbsolutePath() + "/img.jpg");
                                        modelLock.lock();
                                        model.get(0).setImage(image);
                                        setModel(model);
                                        modelLock.unlock();
                                        //robot.setImage(image);
                                        //sendBroadcast(new Intent().setAction(IMAGE_COMPLETE));
                                        currConnectedDevice.disconnect();

                                    } catch (Exception ex) {
                                        Log.e("JPEG.receiver", "Error in writing jpeg");
                                    }
                                    strJSON = null;

                                } else*/

                                if (UPDATE_COMPLETE.equals(action)) {
                                    // update complete
                                    // convert our string into JSON
                                    try {
                                        JSONObject jsonMessage = new JSONObject(strJSON);
                                        // get hash value from json string
                                        CRC32 crc32 = new CRC32();
                                        crc32.update(strJSON.getBytes());
                                        long statusHashValue = crc32.getValue();
                                        strJSON = null;

                                        Log.i("Controller.Update", jsonMessage.toString(2));

                                        // sort closest to furthest
                                        //modelLock.lock();
                                        //Collections.sort(model, new Comparator<Robot>() {
                                        //    @Override
                                        //    public int compare(Robot lhs, Robot rhs) {
                                        //        return rhs.getProximity() - lhs.getProximity();
                                        //    }
                                        //});
                                        //modelLock.unlock();

                                        if (jsonMessage.getString("msgtype").equals("ack")) {
                                            // no update
                                            modelLock.lock();
                                            setModel(model);
                                            modelLock.unlock();
                                            //robotsAsBTDevices.clear();
                                            currConnectedDevice.disconnect();
                                            Log.i("UPDATE.receiver", "ACK!");
                                            return;
                                        }

                                        // get robot's image
                                        String makeOfRobot = jsonMessage.getString("model");
                                        int imgOfBot = -1;
                                        if (makeOfRobot.equals("neato")) {
                                            imgOfBot = R.drawable.svg_neato;
                                        } else if (makeOfRobot.equals("roomba")) {
                                            imgOfBot = R.drawable.svg_discovery;
                                        } else if (makeOfRobot.equals("bender")) {
                                            imgOfBot = R.drawable.svg_bender;
                                        } else if (makeOfRobot.equals("dirtdog")) {
                                            imgOfBot = R.drawable.svg_dirt_dog;
                                        } else if (makeOfRobot.equals("roomba500")) {
                                            imgOfBot = R.drawable.svg_roomba500;
                                        } else if (makeOfRobot.equals("eva")) {
                                            imgOfBot = R.drawable.svg_eva;
                                        } else {
                                            // it's just gotta be junior
                                            imgOfBot = R.drawable.junior;
                                        }

                                        modelLock.lock();
                                        for (Robot bot : model) {
                                            if (bot.getId().equals(currConnectedDevice.getDevice().getAddress())) {
                                                String originalName = bot.getName();
                                                // set this robot's name
                                                bot.setName(jsonMessage.getString("name"));
                                                // set it's state
                                                bot.setCurrState(jsonMessage.getString("state"));
                                                // set it's model
                                                bot.setModel(makeOfRobot);
                                                // set image of robot
                                                bot.setImage(imgOfBot);
                                                // get the robot progression if it exists
                                                if (!(jsonMessage.isNull("progression"))) {
                                                    bot.setProgression(jsonMessage.getJSONArray("progression"));
                                                }
                                                // set checksum value
                                                bot.setStatusHashValue(statusHashValue);
                                                if (originalName == null) {
                                                    if (isCurrRobotVisible) {
                                                        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                                                .format(new Date())
                                                                + ",SETVISIBLE,"
                                                                + bot.getName());
                                                    } else {
                                                        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                                                .format(new Date())
                                                                + ",SETHIDDEN,"
                                                                + bot.getName());
                                                    }
                                                }
                                            }
                                        }

                                        setModel(model);
                                        modelLock.unlock();

                                        //robotsAsBTDevices.clear();
                                        currConnectedDevice.disconnect();

                                        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                                .format(new Date())
                                                + ",UPDATED,"
                                                + jsonMessage.getString("name"));
                                    } catch (JSONException ex) {
                                        currConnectedDevice.disconnect();
                                        StringWriter stringWriter = new StringWriter();
                                        PrintWriter printWriter = new PrintWriter(stringWriter, true);
                                        ex.printStackTrace(printWriter);
                                        Log.e("UPDATE.receiver", stringWriter.toString());
                                    }
                                }
                            }
                        });
                    }
                };


                IntentFilter filter = new IntentFilter();
                //filter.addAction(DISMISS);
                filter.addAction(DESERIALIZE_JPEG);
                filter.addAction(UPDATE_COMPLETE);
                //filter.addAction(SEND_JPEG);
                registerReceiver(receiver, filter);
            }
        }); // end handler post
    }

    @Override
    public int onStartCommand(final Intent intent, final int flags, final int startId) {

        // go on service thread
        serviceHandler.post(new Runnable() {
            @Override
            public void run() {
                onStartCommandSeparateThread();
                controllerServiceIntent = intent;
            }
        });

        return Service.START_STICKY;
    }

    private void onStartCommandSeparateThread() {
        final String TAG = "onStartSeparateThread";

        // enabling bluetooth is not a blocking call so we need to make sure bt is on
        while (!btAdapter.isEnabled()) {
            try {
                btAdapter.enable();
                Log.i(TAG, "Putting service thread to sleep");
                sleep(500);
            } catch (InterruptedException ex) {
                Log.e(TAG, "Service thread failed to sleep.");
            }
        }
        // get scanner for starting and stopping le scanning -> looks like bt has to be enabled first
        leScanner = btAdapter.getBluetoothLeScanner();

        //start scanning
        leScanner.startScan(null,
                new ScanSettings.Builder()
                        .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                        //.setReportDelay(0)
                        //.setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
                        //.setMatchMode(ScanSettings.MATCH_MODE_AGGRESSIVE)
                        //.setNumOfMatches(ScanSettings.MATCH_NUM_MAX_ADVERTISEMENT)
                        .build(),
                scanCallback);
        /*
        while (!(btAdapter.startLeScan(leCallback))) {
            try {
                Log.i(TAG, "Failed to start scan. Re-attempting to start scan");
                sleep(500);
            } catch (InterruptedException ex) {
                Log.e(TAG, "Service thread failed to sleep.");
            }
        } */
        isScanning = true;
    }

    @Override
    public void onDestroy() {

        // stop service
        this.stopService(notifViewIntent);
        Log.i("Controller.onDestroy()", "Stopped NotificationViewService");

        //close if we are connected
        if (currConnectedDevice != null) {
            currConnectedDevice.close();
        }

        //stop scanning
        if (isScanning) {
            serviceHandler.post(new Runnable() {
                @Override
                public void run() {
                    //btAdapter.stopLeScan(leCallback);
                    leScanner.stopScan(scanCallback);
                }
            });
        }


        //turn off user's bluetooth if it was off before app launch
        if (btInitOff) {
            btAdapter.disable();
        }

        unregisterReceiver(receiver);


        // end transfer review if app is closed during transfer process
        if (!statusReviewOff) {
            statusReview.close();
        }

        // end notification read thread if app is closed during transfer process
        if (readNotifications != null) {
            readNotifications.close();
        }

        // destroy any active notifications
        notifManager.cancelAll();

        handleScanCallbacks.close();

        // cleaning up
        //btAdapter = null;
        //uuidOfInterest = null;
        //scanCallback = null;
        //btGattCallback = null;
        //supportedServices = null;
        //supportedCharas = null;
        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                .format(new Date())
                + ",SHUTDOWN,robotlinkapp");
        Log.i("Controller.onDestroy()", "Destroyed");

    }

    /**
     *returns null -> this is a pure started service
     */
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    /**
     * responsible for keeping our service thread alive and referable through serviceHandler
     */
    private class ServiceLooper extends Thread {
        @Override
        public void run() {
            Looper.prepare();
            serviceHandler = new Handler();
            Looper.loop();
        }
    }

    /**
     * instances help enqueue scan callback jobs
     */
    private class ScanCallbackPackage {
        private BluetoothDevice device;
        private int rssi;
        private byte[] scanRecord;

        public ScanCallbackPackage(BluetoothDevice device, int rssi, byte[] scanRecord) {
            this.device = device;
            this.rssi = rssi;
            this.scanRecord = scanRecord;
        }

        public BluetoothDevice getDevice() {
            return device;
        }

        public int getRssi() {
            return rssi;
        }

        public byte[] getScanRecord() {
            return scanRecord;
        }
    }

    /**
     * thread responsible for handling scanCallback packages
     */
    private class HandleScanCallbacks extends Thread {
        final String TAG = "Controller.Callbacks";
        private boolean keepAlive = true;
        private String addressOfRobotNeedingReply = null;

        @Override
        public void run() {
            while (keepAlive) {

                scanCallbackPackagesLock.lock();

                while (scanCallbackPackages.isEmpty() && keepAlive) {
                    scanCallbackPackagesLock.unlock();
                    try {
                        //Log.i(TAG, "About to sleep");
                        sleep(200);
                    } catch (Exception ex) {

                    }
                    scanCallbackPackagesLock.lock();
                }

                if (keepAlive) {

                    ScanCallbackPackage callbackPackage = scanCallbackPackages.remove(0);
                    scanCallbackPackagesLock.unlock();

                    //Log.i(TAG, "Queue size: " + scanCallbackPackages.size());

                    BluetoothDevice device = callbackPackage.getDevice();
                    int rssi = callbackPackage.getRssi();
                    byte scanRecord[] = callbackPackage.getScanRecord();

                    // safety-net : if uuid is null then app should not proceed.
                    if (uuidOfInterest == null) {
                        Log.i(TAG, "Called when uuidOfInterest is null");
                        try {
                            (Looper.myLooper()).getThread().sleep(200);
                            Thread.currentThread().interrupt();
                        } catch (InterruptedException ex) {
                            Log.i(TAG, "Failed to sleep or interrupt");
                        }
                        return;
                    }

                    // If device has been added to rejected list then go no further.
                    if (rejectedDeviceList.contains(device)) {
                        //Log.i(TAG, "Rejected " + device.getAddress());
                        continue;
                    }

                    // so this robot won't be added to robotsAsBTDevices without connecting
                    if (isConnected) {
                        scanCallbackPackages.clear();
                        continue;
                    }

                    // handling dismissed robots
                    // if there's a dismissed robot that has the same bt device in callback then restart the
                    // dismissedRobot's timer
                    //for (DismissedRobot dismissedRobot : dismissedList) {
                    //if (dismissedRobot.getBluetoothDevice().equals(device)) {
                    //Log.i(TAG, "found dismissed robot");
                    //dismissedRobot.restartTimer();
                    //return;
                    //}
                    //}


                    Log.i(TAG, device.getAddress());

                    // check to see if there is a reply job in queue
                    replyQueueLock.lock();
                    if (!(replyQueue.isEmpty())) {
                        // there's a reply job waiting
                        if ((replyQueue.get(0).getRobotId()).equals(device.getAddress())) {
                            // this device is the robot that needs a reply
                            // so connect right away
                            robotsAsBTDevices.put(device, rssi);
                            robotUpdateClock.stopTimer();
                            // needs to be here in case a robot right before took a false
                            isCurrRobotVisible = true;
                            connect(device);
                        }
                    }
                    replyQueueLock.unlock();


                    // check if this device has been connected to recently
                    if (!robotsAsBTDevices.containsKey(device)) {

                        List<String> listOfStructures = parseScanRecord(scanRecord);
                        //Log.i(TAG, device.getAddress());
                        for (String s : listOfStructures) {
                            Log.i(TAG, s);
                        }

                        if (uuidOfInterest.equals(getAdUuidOfPeripheral(listOfStructures))) {
                            //DeviceUtilities item = new DeviceUtilities(device, rssi);
                            isCurrRobotVisible = isRobotVisible(listOfStructures);

                            // check if there is an update with this robot
                            if (checkForUpdate(device.getAddress(), listOfStructures)) {
                                robotsAsBTDevices.put(device, rssi);
                                robotUpdateClock.stopTimer();
                                //connect and get info
                                connect(device);
                            } else {
                                //***need to set visibility because there is no update***
                                modelLock.lock();
                                model = getModel();
                                // find robot in model
                                for (Robot bot : model) {
                                    if (bot.getId().equals(device.getAddress())) {
                                        int index = model.indexOf(bot);
                                        Robot robotPrime = (Robot)bot.clone();
                                        robotPrime.setProximity(rssi);
                                        robotPrime.setVisible(isCurrRobotVisible);
                                        model.set(index, robotPrime);
                                        // set the model now
                                        setModel(model);
                                        if (bot.isVisible() && !isCurrRobotVisible) {
                                            Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                                    .format(new Date())
                                                    + ",SETHIDDEN,"
                                                    + bot.getName());
                                        } else if (!bot.isVisible() && isCurrRobotVisible) {
                                            Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                                    .format(new Date())
                                                    + ",SETVISIBLE,"
                                                    + bot.getName());
                                        }
                                        break;
                                    }
                                }
                                modelLock.unlock();
                            }

                        } else {
                            // Add to rejected list
                            rejectedDeviceList.add(device);
                        }

                    } else { //this is one of our robots

                        //Log.i(TAG, "Heard one of our robots");

                        // update proximity
                        /*
                        modelLock.lock();
                        model = getModel();
                        // check if current robot is already in our model
                        for (Robot bot : model) {
                            if (bot.getId().equals(device.getAddress())) {
                                bot.setProximity(rssi);
                                model.set(model.indexOf(bot), (Robot) bot.clone());
                                break;
                            }
                        }
                        setModel(model);
                        modelLock.unlock();
                        */


            /*
            // update rssi values
            for (BluetoothDevice dev : robotsAsBTDevices.keySet()) {

                //DeviceUtilities dev = (arAdapter.getItem(i));

                //if (((dev.getItem()).getAddress()).equals(device.getAddress())) {
                if (dev.getAddress().equals(device.getAddress())) {

                    //dev.addToRssiList(rssi);

                    robotsAsBTDevices.get(dev).add(rssi);

                    if (robotsAsBTDevices.get(dev).size() > 9) {
                        /*
                        int n = 0;
                        for (Integer x :robotsAsBTDevices.get(dev)) {
                            n += x;
                        }
                        n = n / 10;
                        dev.setRssi(n);
                        //arAdapter.notifyDataSetChanged();
                        dev.clearRssiList();

                        smartConnect();
                    }
                    break;
                }
            }*/
                    }
                }
            }

            if (scanCallbackPackagesLock.isHeldByCurrentThread()) {
                scanCallbackPackagesLock.unlock();
            }
        }

        public void close() {
            keepAlive = false;
        }
    }

    /**
     * @param scanRecord is  payload
     * @return arraylist of all data structures in payload
     */
    private List<String> parseScanRecord(byte[] scanRecord) {
        // Contains info in payload
        List<String> GAPStructures = new ArrayList<>();

        // Keeping track of which byte we're on
        int index = 0;

        // Loop through whole payload
        while (index < scanRecord.length) {

            int lengthOfGAPStructure = scanRecord[index];

            // checking to see if payload is not full
            if (lengthOfGAPStructure == 0) {
                // if here then payload no longer contains any ad structures.
                break;
            }
            index++;

            //checking to see if GAP type is 0
            if (scanRecord[index] == 0) {
                // if here then no specified GAP type was used
                break;
            }


            //String structure = (Arrays.copyOfRange(scanRecord, index, ((index+lengthOfGAPStructure)-1))).toString();
            //loop through each byte and get it's binary representation in string form******
            byte[] adData = Arrays.copyOfRange(scanRecord, index, ((index + lengthOfGAPStructure)));
            String structure = "";
            for (int i = 0; i < adData.length; i++) {
                structure += ((Integer.toBinaryString((adData[i] & 0b11111111) + 256)).substring(1) + " ");
            }

            GAPStructures.add(structure);
            index += (lengthOfGAPStructure);

        }
        return GAPStructures;
    }


    /**
     * @param listOfStructures is an arraylist with each element being a complete payload structure
     * @return the uuid found at 0x06 gap data structure. if structure not found then a null uuid is returned
     */
    private UUID getAdUuidOfPeripheral(List<String> listOfStructures) {

        UUID uuidTemp = null;
        boolean incListOfUuidFound = false;
        String strOfUuidElements = "";

        for (String s : listOfStructures) {

            //Log.i("getAdUuidOfPeripheral()", "Checking = " + s.substring(0, 8));

            if ((s.substring(0, 8)).equals("00000110")) {

                incListOfUuidFound = true;
                strOfUuidElements = s.substring(9);
                //Log.i("getAdUuidOfPeripheral()", "Found incomplete list of 128 bit UUIDs");
                //Log.i("getAdUuidOfPeripheral()", "Bytes of UUID = " + strOfUuidElements);
                //Log.i("getAdUuidOfPeripheral()", "Length = " + byteStringOfUuid);
                break;

            }
        }

        if (!incListOfUuidFound) {
            return null;
        }

        strOfUuidElements = changeByteOrder(strOfUuidElements);
        //Log.i("getAdUuidOfPeripheral()", "Reordered bytes of UUID = " + strOfUuidElements);

        strOfUuidElements = binaryStrToUuidStr(strOfUuidElements);
        //Log.i("getAdUuidOfPeripheral()", "UUID = " + strOfUuidElements);

        uuidTemp = UUID.fromString(strOfUuidElements);
        return uuidTemp;
    }

    /**
     * takes a look at the 0xFF structure which will tell if the robot
     * should be displayed or not
     * 0 = visible
     * 1 = not visible
     * @param listOfStructures is an arraylist contating all gap structures
     * @returns true if robot should be displayed to user
     */
    private boolean isRobotVisible(List<String> listOfStructures) {

        for (String s : listOfStructures) {
            //Log.i("shouldIgnoreRobot()", "Checking = " + s.substring(0, 8));
            if ((s.substring(0, 8)).equals("11111111")) {
                //Log.i("isRobotVisible()", "Length: " + ((s.substring(9)).length()));
                if ((s.charAt(34)) == '0') {
                    return true;
                }
                return false;
                //Log.i("shouldIgnoreRobot()", "Bit is: " + (s.charAt(s.length() - 2));
            }
        }
        return true;
    }

    private boolean checkForUpdate(String addressOfRobot, List<String> listOfStructures) {
        // check if we already have a robot with this address
        modelLock.lock();
        Robot robot = null;
        model = getModel();
        for (Robot bot : model) {
            if (bot.getId().equals(addressOfRobot)) {
                robot = bot;
                break;
            }
        }
        modelLock.unlock();
        // no robot has been found in current model -> need to connect
        if (robot == null) {
            return true;
        }

        Log.i("Controller.checkUpdate", "Robot checksum: " + robot.getStatusHashValue());
        // get checksum value from server
        for (String s : listOfStructures) {

            if ((s.substring(0, 8)).equals("11111111")) {

                Log.i("Controller.checkUpdate", "Ad checksum: "
                        + Long.parseLong(s.substring(36).replaceAll("\\s",""), 2) + "");
                // compare against what the last checksum was for this robot
                if (robot.getStatusHashValue() != Long.parseLong(s.substring(36).replaceAll("\\s",""), 2)) {
                    return true;
                }
                break;
                //Log.i("Controller.checkUpdate", Long.toHexString(Long.parseLong(s.substring(36).replaceAll("\\s",""), 2)) + "");
                //Log.i("Controller.checkUpdate", s.substring(36).replaceAll("\\s","").length() + "");
            }
        }

        return false;
    }

    /**
     * @param byteString is a uuid Byte String in little endian
     * @return uuid byte string in big endian
     */
    private String changeByteOrder(String byteString) {

        String result = "";


        int end = byteString.length() - 5;
        int start = end - 4;

        //if even then account for a space
        int whiteSpaceTracker = 1;

        for (int i = 0; i < 32; i++) {

            result = (result + byteString.substring(start, end) + " ");

            if ((whiteSpaceTracker % 2) == 1) {
                start = end;
                end = end + 4;
            } else {

                start = start - 13;
                end = start + 4;
            }
            whiteSpaceTracker++;
        }

        return result;


    }

    /**
     * @param binaryString big endian byte string of uuid
     * @return hex string of uuid
     */
    private String binaryStrToUuidStr(String binaryString) {

        String result = "";

        int start = 0;
        int end = 4;

        for (int i = 0; i < 32; i++) {

            result += Integer.toHexString(Integer.parseInt(binaryString.substring(start, end), 2));
            if (i == 7 || i == 11 | i == 15 || i == 19) {
                result += "-";
            }
            start = end + 1;
            end = end + 5;
        }

        return result;

    }

    /**
     * stops scanning and connects to passed device
     */
    private void connect(BluetoothDevice device) {

        //stop scanning
        if (isScanning) {
            //btAdapter.stopLeScan(leCallback);
            try {
                leScanner.stopScan(scanCallback);
            } catch (Exception ex) {
                serviceHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        onStartCommandSeparateThread();
                    }
                });
            } finally {
                isScanning = false;
            }
        }

        // if we're already connected get out
        if (isConnected) {
            return;
        }

        Log.i("Controller.connect()", "connecting to " + device.getAddress());

        try {
            // using reflection to get internal connectGatt
            Method connectGattMethod = device.getClass().getMethod("connectGatt", Context.class, boolean.class, BluetoothGattCallback.class, int.class);
            connectGattMethod.invoke(device, ControllerService.this,
                    false, btGattCallback, BluetoothDevice.TRANSPORT_LE);
        } catch (Exception ex) {
            Log.i("Controller.connect()", "failed connection");
        }
        // clear queue of callback packages
        scanCallbackPackages.clear();

        isConnected = true;

        /*
        Log.i("smartConnect()", "Just got in");
        int bestRssi = (arAdapter.getItem(0)).getRssi();
        int index = 0;
        //int[] arOfRssi = new int [arAdapter.getCount()];
        for (int i = 0; i < arAdapter.getCount(); i++) {

            //maxRssi = (arAdapter.getItem(i)).getRssi();
            if (bestRssi < (arAdapter.getItem(i)).getRssi()) {
                bestRssi = (arAdapter.getItem(i)).getRssi();
                index = i;
            }
        }
        //if (bestRssi >= -65) {

        if (robot.getGattServer() == null) {

            // checking for android version
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                //device is running marshmallow or higher
                serviceHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                            leScanner.stopScan(scanCallback);
                        }
                    }
                });
            } else {
                //device is running either jellybean, kitkat, or lollipop
                serviceHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        btAdapter.stopLeScan(leCallback);
                    }
                });
            }
            isScanning = false;

            // using reflection to get internal connectGatt
            BluetoothDevice device = ((arAdapter.getItem(index)).getItem());
            try {
                Method connectGattMethod = device.getClass().getMethod("connectGatt", Context.class, boolean.class, BluetoothGattCallback.class, int.class);
                robot.setGattServer((BluetoothGatt) connectGattMethod.invoke(device, LinkService.this,
                        false, btGattCallback, BluetoothDevice.TRANSPORT_LE));
            } catch (Exception ex) {

            }


            Log.i("smartConnect()", "After connectGatt");

        }
        //} else {
        // notify to move closer
        //broadcastStatus(1);
        //}

        */
    }

    /**
     * used for multi packet reads.
     * compares time stamps to determine when notification wave is over
     */

    private class TransferStatusReview extends Thread {

        private boolean keepAlive;
        long transferTimeStamp;

        public TransferStatusReview() {
            keepAlive = true;
        }

        @Override
        public void run() {

            while (keepAlive) {

                // take time at beginning of loop
                transferTimeStamp = System.currentTimeMillis();

                // sleep for a bit
                try {
                    sleep(100);
                    //Log.i("transfer", "sleeping");
                } catch (InterruptedException ex) {
                }

                // take an interruptible lock
                try {
                    transferLock.lockInterruptibly();
                } catch (InterruptedException ex) {
                    Log.i("TransferStatusReview", "Closed while blocking");
                }

                Log.i("TransferReview", (transferTimeStamp - notifyTimeStamp) + "");

                if ((transferTimeStamp - notifyTimeStamp) >= 250 && packetsFound.containsValue("")) {
                    //TRANSFER FINISHED

                    // gather missing packet numbers
                    byte[] missedPackets = new byte[20];
                    short byteIndex = 0;
                    String bitString = "1"; // indicates missing packet list
                    for (Integer i = 0; i < packetsFound.size(); i++) {

                        if (bitString.length() == 8) {
                            missedPackets[byteIndex] = (byte) Integer.parseInt(bitString, 2);
                            byteIndex++;
                            Log.i("TransferReview", "Missed packets: " + bitString);
                            bitString = "";
                        }

                        if (packetsFound.get(i).equals("")) {
                            bitString += "1";
                            //Log.i("Transfer null", packetsFound.get(i));
                        } else {
                            //Log.i("Transfer string", packetsFound.get(i));
                            bitString += "0";
                        }

                    }
                    // append 0's to remaining bits of last byte
                    for (int i = bitString.length(); i < 8; i++) {
                        bitString += "0";
                    }

                    missedPackets[byteIndex] = (byte) Integer.parseInt(bitString, 2);

                    // write missing packet list back to server
                    missingPacketWrite.setValue(missedPackets);
                    currConnectedDevice.writeCharacteristic(missingPacketWrite);

                    //not alive!! let others know of your suicide.
                    statusReviewOff = true;

                    // stop loop
                    keepAlive = false;

                    // expecting missed packets
                    awaitingMissedPackets = true;
                }

                //release lock
                transferLock.unlock();
            }

        }

        public void close() {
            keepAlive = false;
            statusReviewOff = true;
        }
    }

    /**
     * * makes a robot and adds it to our copy of the model
     * * takes care of enabling notifications
     */
    private void makeRobot() {

        try {
            //ArrayList<BluetoothGattCharacteristic> allSupportedCharacteristics = new ArrayList<>();
            ArrayList<BluetoothGattService> serviceList = (ArrayList<BluetoothGattService>)currConnectedDevice.getServices();

            // characteristic we read from to start notifications
            BluetoothGattCharacteristic packetRead = null;

            // characteristic that indicates missed packets
            BluetoothGattCharacteristic missingPacketRead = null;

            // get all supported characteristics from the services
            for (BluetoothGattService service : serviceList) {

                String uuidOfService = service.getUuid().toString();

                if (supportedServices.containsKey(uuidOfService)) {

                    ArrayList<BluetoothGattCharacteristic> charList = ((ArrayList<BluetoothGattCharacteristic>) service.getCharacteristics());

                    for (BluetoothGattCharacteristic chara : charList) {

                        String uuidOfCharacteristic = chara.getUuid().toString();

                        if (supportedCharas.containsKey(uuidOfCharacteristic)) {

                            /*
                            // checking if characteristic supports notification or indication (one or the other)
                            if ((chara.getProperties() & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0) {

                                //enable notifications
                                subscribe(chara, true, 0, currConnectedDevice);
                                //Log.i("makeRobot().not", supportedCharas.get(uuidOfCharacteristic));
                                try {
                                    makeRobotBlock.take();
                                } catch (InterruptedException ex) {
                                    Log.e("makeRobot()", "failed to take from blocking queue");
                                }

                            } else if ((chara.getProperties() & BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0) {

                                //enable indications
                                subscribe(chara, true, 1, currConnectedDevice);
                                //Log.i("makeRobot().ind", supportedCharas.get(uuidOfCharacteristic));
                                try {
                                    makeRobotBlock.take();
                                } catch (InterruptedException ex) {
                                    Log.e("makeRobot()", "failed to take from blocking queue");
                                }
                            }
                            */


                            if (supportedCharas.get(uuidOfCharacteristic).equals("Total Number of Packets")) {
                                totalNumOfPacketsWrite = chara;
                            } else if (supportedCharas.get(uuidOfCharacteristic).equals("Packet Write")) {
                                packetWrite = chara;
                            } else if (supportedCharas.get(uuidOfCharacteristic).equals("Missing Packet Write")) {
                                missingPacketWrite = chara;
                            } else if (supportedCharas.get(uuidOfCharacteristic).equals("Packet Read")) {
                                packetRead = chara;
                            } else if (supportedCharas.get(uuidOfCharacteristic).equals("Missing Packet Read")) {
                                missingPacketRead = chara;
                            }

                            //allSupportedCharacteristics.add(chara);
                        }
                    }
                }
            }

            //boolean sentReply = false; //patch fix
            // handle reply to robot if there is one in the reply queue
            replyQueueLock.lock();
            if (!(replyQueue.isEmpty())) {
                if ((replyQueue.get(0).getRobotId()).equals(currConnectedDevice.getDevice().getAddress())) {

                    // get reply package
                    ReplyPackage replyPackage = replyQueue.remove(0);
                    replyQueueLock.unlock();

                    //enable indications
                    subscribe(missingPacketRead, true, 1, currConnectedDevice);
                    try {
                        makeRobotBlock.take();
                    } catch (InterruptedException ex) {
                        Log.e("makeRobot()", "failed to take from blocking queue");
                    }

                    // call handleReplyMessage to send reply to robot
                    handleReplyMessage(replyPackage);
                    // when server has confirmed it received the entire message
                    try {
                        makeRobotBlock.take();
                    } catch (Exception ex) {
                        Log.e("makeRobot()", "failed to take from blocking queue");
                    }
                    //sentReply = true; // patch fix
                    replyQueueLock.lock();
                }
            }
            replyQueueLock.unlock();

            // patch fix
            /*if (sentReply) {

                }
                currConnectedDevice.disconnect();
                return;
            }*/


            //enable notifications
            subscribe(packetRead, true, 0, currConnectedDevice);
            try {
                makeRobotBlock.take();
            } catch (InterruptedException ex) {Log.e("makeRobot()", "failed to take from blocking queue");Log.e("makeRobot()", "failed to take from blocking queue");
                Log.e("makeRobot()", "failed to take from blocking queue");
            }

            // read packetRead characteristic to start notifications
            if (currConnectedDevice.readCharacteristic(packetRead)) {
                try {
                    makeRobotBlock.take();
                } catch (Exception ex) {
                    Log.e("makeRobot()", "failed to take from blocking queue");
                }
            }





            /*
            for (BluetoothGattCharacteristic chara : allSupportedCharacteristics) {

                String uuidOfCharacteristic = chara.getUuid().toString();

                if (supportedCharas.get(uuidOfCharacteristic).equals("Missing Packet Read")) {
                    continue;
                }

                // Attempt to read this characteristic
                if (currConnectedDevice.readCharacteristic(chara)) {
                    try {
                        makeRobotBlock.take();
                    } catch (Exception ex) {
                        Log.e("makeRobot()", "failed to take from blocking queue");
                    }
                }


                //Log.i("Controller.makeRobot()", supportedServices.get(uuidOfService)
                //       + ": " + supportedCharas.get(uuidOfCharacteristic) +
                //      " = " +(canBeRead ? getCharaValue(chara) : "Cannot be read"));

            } */



        /*
        int customServiceCount = 0;
        for (BluetoothGattService service : serviceList) {

            String uuidOfService = service.getUuid().toString();

            ArrayList<BluetoothGattCharacteristic> charList = ((ArrayList<BluetoothGattCharacteristic>) service.getCharacteristics());
            List<RobotCharacteristic> robotCharacteristics = new ArrayList<>();
            int customCharaCount = 0;

            for (BluetoothGattCharacteristic chara : charList) {


                String uuidOfCharacteristic = chara.getUuid().toString();
                boolean canBeRead = false;

                // Attempt to read this characteristic
                if (robot.getGattServer().readCharacteristic(chara)) {
                    try {
                        chara = DeviceUtilities.blockQueue.take();
                        canBeRead = true;
                    } catch (Exception ex) {
                        Log.e("makeRobot()", "failed to take from blocking queue");
                    }
                }

                if (canBeRead) {
                    // checking if characteristic supports notification or indication (one or the other)
                    if ((chara.getProperties() & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0) {

                        //enable notifications
                        subscribe(chara, true, 0);

                        try {
                            makeRobotBlock.take();
                        } catch (InterruptedException ex) {
                            Log.e("makeRobot()", "failed to take from blocking queue");
                        }

                    } else if ((chara.getProperties() & BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0) {

                        //enable indications
                        subscribe(chara, true, 1);
                    }
                }

                if (supportedCharas.containsKey(uuidOfCharacteristic)) {
                    robotCharacteristics.add(new RobotCharacteristic(
                            supportedCharas.get(uuidOfCharacteristic),
                            (canBeRead ? getCharaValue(chara) : "Cannot be read"),
                            chara
                    ));
                } else {
                    robotCharacteristics.add(new RobotCharacteristic(
                            "Custom Characteristic " + (++customCharaCount),
                            (canBeRead ? getCharaValue(chara) : "Cannot be read"),
                            chara
                    ));
                    Log.i("makeRobot()", uuidOfCharacteristic);
                }
            }

            robot.addRobotService(
                    (supportedServices.containsKey(uuidOfService) ?
                            supportedServices.get(uuidOfService) : "Custom Service " + (++customServiceCount)),
                    service, robotCharacteristics
            );
        }
        */

            // says if robot is already known to the model
            boolean alreadyContained = false;
            modelLock.lock();
            model = getModel();
            // check if current robot is already in our model
            for (Robot bot : model) {
                if (bot.getId().equals(currConnectedDevice.getDevice().getAddress())) {
                    int index = model.indexOf(bot);
                    Robot robotPrime = (Robot)bot.clone();
                    robotPrime.setProximity(robotsAsBTDevices.get(currConnectedDevice.getDevice()));
                    robotPrime.setVisible(isCurrRobotVisible);
                    model.set(index, robotPrime);
                    alreadyContained = true;
                    if (bot.isVisible() && !isCurrRobotVisible) {
                        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                .format(new Date())
                                + ",SETHIDDEN,"
                                + bot.getName());
                    } else if (!bot.isVisible() && isCurrRobotVisible) {
                        Log(new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss")
                                .format(new Date())
                                + ",SETVISIBLE,"
                                + bot.getName());
                    }
                    break;
                }
            }
            modelLock.unlock();

            if (!alreadyContained) {
                // this is a new robot
                // setting robot name, rssi (proximity) and ID
                Robot robot = new Robot(robotsAsBTDevices.get(currConnectedDevice.getDevice()),
                        currConnectedDevice.getDevice().getAddress());
                robot.setVisible(isCurrRobotVisible);
                //robot.setImage(R.drawable.svg_neato); //TEMPORARY
                modelLock.lock();
                model.add(robot);
                modelLock.unlock();
            }

        } catch (NullPointerException ex) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter, true);
            ex.printStackTrace(printWriter);
            Log.e("makeRobot()", stringWriter.toString());
            return;
        }

        // thread for reading notifications
        readNotifications = new ReadNotifications();
        readNotifications.start();
        Log.i("Controller.makeRobot()", "Finished");
    }

    /**
     * enables or disables characteristic notification/indication
     *
     * @param characteristic   will have its notify descriptor turned on or off
     * @param enable           specifies to enable or disable notification
     * @param notifyOrIndicate specifies an indication or notification: notify = 0, indicate = 1
     * @param gatt             is server of robot
     */
    private void subscribe(BluetoothGattCharacteristic characteristic,
                           boolean enable, int notifyOrIndicate, BluetoothGatt gatt) {

        if (gatt == null) {
            Log.i("Controller.subscribe()", "gatt server is null");
            return;
        }
        // gatt server needs to know to notify or not
        gatt.setCharacteristicNotification(characteristic, enable);

        // characteristic needs to know to notify or not

        // uuid of descriptor responsible for notification/indication enabling/disabling specified
        // by the Bluetooth spec - org.bluetooth.descriptor.gatt.client_characteristic_configuration
        UUID descripUuid = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");


        if (notifyOrIndicate == 0) {
            //setting Notification value
            (characteristic.getDescriptor(descripUuid)).setValue((enable ?
                    BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE :
                    BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE));
        } else {
            //setting Indication value
            (characteristic.getDescriptor(descripUuid)).setValue((enable ?
                    BluetoothGattDescriptor.ENABLE_INDICATION_VALUE :
                    BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE));
        }


        //write descriptor - received in callback onDescriptorWrite
        gatt.writeDescriptor(characteristic.getDescriptor(descripUuid));
    }
    /**
     * **useful for initial reads**
     * checks format types to determine how to interpret the characteristic's value
     *
     * @param chara is the characteristic
     * @return string representation of the characteristic value
     */
    private String getCharaValue(BluetoothGattCharacteristic chara) {

        if (chara.getUuid().toString().equals("00002a10-30de-4630-9b59-27228d45bf11")) {

            // grabbing initial value in characteristic which says how many packets total
            // there are to be sent
            int totalNumOfPackets = java.nio.ByteBuffer.wrap(chara.getValue()).getInt();
            Log.i("getCharaValue", ((Integer)totalNumOfPackets).toString());
            // initialize map with correct number of packets wanted
            if (totalNumOfPackets >= 128) {

                for (int i = 0; i < 128; i++) {
                    packetsFound.put(i, "");
                }

            } else {

                for (int i = 0; i < totalNumOfPackets; i++) {
                    packetsFound.put(i, "");
                }
            }

            return ((Integer)totalNumOfPackets).toString();

        } else if (chara.getIntValue(BluetoothGattCharacteristic.FORMAT_UINT16, 0) == 128) {

            //this is a generic computer as documented in the bluetooth specification
            return "Generic Computer";

        } else {

            return chara.getStringValue(0);

        }
    }

    /**
     * thread responsible for handling notifications from the queue
     */
    private class ReadNotifications extends Thread {

        private boolean keepAlive = true;

        @Override
        public void run() {

            while (keepAlive) {

                notificationQueueLock.lock();

                while (notificationQueue.isEmpty() && keepAlive) {
                    Log.i("Controller.Read", "Queue is empty: " + notificationQueue.size());
                    notificationQueueLock.unlock();
                    try {
                        sleep(10);
                    } catch (Exception ex) {
                        Log.e("Controller.Read", "Failed sleep");
                    }
                    notificationQueueLock.lock();
                }

                if (keepAlive) {

                    //Log.i("Controller.Read", "Starting notif read: " + notificationQueue.size());

                    // update time stamp
                    notifyTimeStamp = System.currentTimeMillis();

                    // take lock - this characteristic is sending an update
                    transferLock.lock();

                    // check if our transfer review is off
                    if (statusReviewOff) {
                        // turn it on
                        statusReview = new TransferStatusReview();
                        statusReview.start();
                        statusReviewOff = false;
                    }

                    byte rawPacket[] = notificationQueue.remove(0);
                    notificationQueueLock.unlock();

                    // getting missing packet flag value
                    int missingFlag = (rawPacket[0] & 0x80);

                    //
                    if ((missingFlag == 128 && awaitingMissedPackets) ||
                            missingFlag == 0 && !awaitingMissedPackets) {


                        // getting packet number data
                        int packetNum = (rawPacket[0] & 0x7F);

                        //testTotal += 1;
                        Log.i("Controller.Read", "Packet number: " + packetNum);
                        Log.i("Controller.Read", "Total Number of Packets: " + totalNumOfPackets);
                        // getting json data
                        byte json[] = new byte[rawPacket.length - 1];
                        System.arraycopy(rawPacket, 1, json, 0, (rawPacket.length - 1));

                        packetsFound.put(packetNum, new String(json));

                        // check to see if all packets in current wave are buffered
                        if (!packetsFound.containsValue("")) {
                            // set awaitingMissedPackets to false
                            awaitingMissedPackets = false;

                            //Log.i("Controller.Read", "no null found");

                            // if here then all packets have been found
                            // start appending strJSON

                            //Log.i("Controller.Read", "size: " + packetsFound.size());

                            for (Integer i = 0; i < packetsFound.size(); i++) {

                                // quick fix to check for interesting null packets
                                //if (packetsFound.get(i).equals("")) {
                                //continue;
                                //}

                    /*
                    //if stringJPEG contains no textual content of characteristic yet
                    if (strJSON == null) {
                        if (packetsFound.get(i).substring(0, 3).equals("$$$")) {
                            strJSON = packetsFound.get(i).substring(3);
                            Log.i("handleJPEG()", "Started JSON string");
                        }
                        Log.i("handleJPEG()", packetsFound.get(i));

                    } else {
                        Log.i("handleJPEG()", "Size: " + packetsFound.size());
                        Log.i("handleJPEG()", "i: " + i);
                        Log.i("handleJPEG()", "Content at i: " + packetsFound.get(i));
                        int start = (packetsFound.get(i).length() - 3);
                        int end = packetsFound.get(i).length();

                        if (packetsFound.get(i).substring(start, end).equals("$$$")) {

                            strJSON += packetsFound.get(i).substring(0, start);
                            // end of JSON

                        } else {
                            //Log.i("handleJPEG()", "adding: " + packetsFound.get(i));
                            strJSON += packetsFound.get(i);

                        }
                    }
                    */

                                if (null == strJSON) {
                                    strJSON = packetsFound.get(i);
                                } else {
                                    strJSON += packetsFound.get(i);
                                }
                                Log.i("Controller.Read", strJSON);
                            }

                            // using RobotCharacteristic's charaValue to keep track of how many packets are left
                            // note: there is no writing to the characteristic and the value stored is local
                            // **there is a smarter more efficient way**
                            //int totalPacketsLeft = Integer.parseInt(robot.getRobotCharacteristic(characteristic).getCharaValue());
                            // totalPacketsLeft = (totalPacketsLeft - packetsFound.size());
                            // robot.getRobotCharacteristic(characteristic).setCharaValue(totalPacketsLeft + "");


                            // writing to service to let it know all packets have been received
                            byte[] successMessage = new byte[20];
                            String bitString = "0"; // indicates success
                            successMessage[0] = (byte) Integer.parseInt(bitString, 2);

                            missingPacketWrite.setValue(successMessage);
                            currConnectedDevice.writeCharacteristic(missingPacketWrite);

                            totalNumOfPackets = (totalNumOfPackets - packetsFound.size());

                            // set up map with correct number of packets wanted
                            packetsFound.clear();
                            if (totalNumOfPackets >= 128) {
                                statusReview.close();
                                notificationQueue.clear();
                                Log.i("Controller.Read", "Before block: 128");
                                try {
                                    characteristicWriteBlock.take();
                                } catch (Exception ex) {

                                }
                                Log.i("Controller.Read", "After block: 128");
                                for (int i = 0; i < 128; i++) {
                                    packetsFound.put(i, "");
                                }

                            } else if (totalNumOfPackets == 0) {

                                //Log.i("Controller.Read", "Ended JSON string");
                                //Log.i("json", strJSON);
                                statusReview.close();
                                close();
                                notificationQueue.clear();
                                // make sure characteristic has been written to
                                Log.i("Controller.Read", "Before block: 0");
                                try {
                                    characteristicWriteBlock.take();
                                } catch (Exception ex) {}
                                Log.i("Controller.Read", "After block: 0");
                                sendBroadcast(new Intent().setAction(UPDATE_COMPLETE));

                            } else {
                                statusReview.close();
                                notificationQueue.clear();
                                Log.i("Controller.Read", "Before block: <128");
                                try {
                                    characteristicWriteBlock.take();
                                } catch (Exception ex) {}
                                Log.i("Controller.Read", "After block: <128");
                                for (int i = 0; i < totalNumOfPackets; i++) {
                                    packetsFound.put(i, "");
                                }
                            }

                        }
                    }

                    // release lock
                    transferLock.unlock();
                }
            }

            if (notificationQueueLock.isHeldByCurrentThread()) {
                notificationQueueLock.unlock();
            }
        }

        public void close() {
            keepAlive = false;
        }
    }


    /**
     * class responsible for making sure all robots in area get updated
     * by clearing robotsAsBTDevices.
     */

    private class RobotUpdateClock {
        //timer object
        private Timer timer;

        public RobotUpdateClock() {
            timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    robotsAsBTDevices.clear();
                }
            }, 2000);
        }

        public void stopTimer() {
            timer.cancel();
        }

        public void startTimer() {
            timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    robotsAsBTDevices.clear();
                }
            }, 2000);
        }
    }


    /**
     * used by views to let controller know to make a reply to a robot
     * @param robotId is the mac address associated with the robot
     * @param msgId is the identifier number associated with the progression element
     * @param responseId is response id of the button the user tapped
     */
    public static void addToReplyQueue(String robotId, String msgId, String responseId) {


        //Log.i("Controller.addReply", robotId + " " + msgId + " " + responseId.toString());
        replyQueueLock.lock();
        replyQueue.add(new ReplyPackage(robotId, msgId, responseId));
        replyQueueLock.unlock();
    }

    /**
     * used to package all relevant data for a reply to a robot
     */
    private static class ReplyPackage {
        private String robotId;
        private String msgId;
        private String responseId;

        public ReplyPackage(String robotId, String msgId, String responseId) {
            this.robotId = robotId;
            this.msgId = msgId;
            this.responseId = responseId;
        }

        public String getResponseId() {
            return responseId;
        }

        public String getMsgId() {
            return msgId;
        }

        public String getRobotId() {
            return robotId;
        }
    }


    /**
     * responsible for sending reply message to server
     * @param replyPackage contains the unique contents for the reply
     */
    private void handleReplyMessage(ReplyPackage replyPackage) {
        final String TAG = "Controller.reply()";

        /**
         * prepare json reply
         */
        JSONObject jsonReply = new JSONObject();
        try {
            jsonReply.put("msgtype", "reply");

            JSONObject msgIdAndResponseId = new JSONObject();
            msgIdAndResponseId.put("id", replyPackage.getMsgId());
            msgIdAndResponseId.put("value", replyPackage.getResponseId());

            JSONArray responsesArray = new JSONArray();
            responsesArray.put(0, msgIdAndResponseId);

            jsonReply.put("responses", responsesArray);

            Log.i(TAG, jsonReply.toString(2));
        } catch (JSONException ex) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter, true);
            ex.printStackTrace(printWriter);
            Log.e(TAG, stringWriter.toString());
        }

        /**
         * convert reply into byte array
         */
        byte[] byteReply = (jsonReply.toString().getBytes(StandardCharsets.UTF_8));
        Log.i(TAG, byteReply.length + ", is the length of byte array");

        /**
         * send total number of packets to server
         */
        int totalNumberOfPackets = (int) Math.ceil(((double) byteReply.length / 19));
        Log.i(TAG, totalNumberOfPackets + " is the number of packets going to be sent");

        byte[] byteTotal = {(byte) (totalNumberOfPackets & 0xFF)};
        totalNumOfPacketsWrite.setValue(byteTotal);
        currConnectedDevice.writeCharacteristic(totalNumOfPacketsWrite);
        try {
            characteristicWriteBlock.take();
        } catch (InterruptedException ex) {
            Log.e(TAG, "Failed to take from blocking queue");
        }
        Log.i(TAG, byteTotal[0] + " after block");


        /**
         * prepare packets and send them to server one at a time
         */
        int totalNumOfBytes = byteReply.length;
        for (int i = 0; i < totalNumberOfPackets; i++) {

            if (totalNumOfBytes >= 20) {
                byte[] packetSend = new byte[20];
                packetSend[0] = (byte) (i & 0xFF);
                System.arraycopy(byteReply, i * 19, packetSend, 1, (packetSend.length - 1));
                Log.i(TAG, new String(packetSend));

                outgoingPackets.put(i, packetSend);
                packetWrite.setValue(packetSend);
                currConnectedDevice.writeCharacteristic(packetWrite);
                try {
                    characteristicWriteBlock.take();
                } catch (InterruptedException ex) {
                    Log.e(TAG, "Failed to take from blocking queue");
                }

                totalNumOfBytes = (totalNumOfBytes - 19);

            } else {

                byte[] packetSend = new byte[totalNumOfBytes + 1];
                packetSend[0] = (byte) (i & 0xFF);
                System.arraycopy(byteReply, i * 19, packetSend, 1, (packetSend.length - 1));
                Log.i(TAG, new String(packetSend));

                outgoingPackets.put(i, packetSend);
                packetWrite.setValue(packetSend);
                currConnectedDevice.writeCharacteristic(packetWrite);
                try {
                    characteristicWriteBlock.take();
                } catch (InterruptedException ex) {
                    Log.e(TAG, "Failed to take from blocking queue");
                }
            }
        }
    }


    /**
     * responsible for sending packets that the server missed in the initial send
     * @param missingPacketNumbers is a byte array indicating which packets were missed
     */

    private void handleMissingPacketRequest(byte[] missingPacketNumbers) {

        String strMissingPacketNumbers = "";
                for (byte b : missingPacketNumbers) {
                    strMissingPacketNumbers+=Integer
                            .toBinaryString((b & 0b11111111) + 256).substring(1);
                }

        for (int i = 0; i < outgoingPackets.size(); i++) {
            if (strMissingPacketNumbers.charAt(i+1) == '1') {
                packetWrite.setValue(outgoingPackets.get(i));
                currConnectedDevice.writeCharacteristic(packetWrite);
                Log.i("handleMissingPackets", "before block");
                try {
                    characteristicWriteBlock.take();
                } catch (InterruptedException ex) {
                    Log.e("handleMissingPackets", "Failed to take from blocking queue");
                }
                Log.i("handleMissingPackets", "after block");
            }
        }
    }


    /**
     * used by the application to log user activity
     * @param logText is the entry to be appended to the log
     */
    public static void Log(String logText) {

        File file = new File("sdcard/Download/data/" + dateAndTimeOfAppStart + "_robotnexus_log.csv");

        if (!file.exists()) {
            try {
                file.createNewFile();
                BufferedWriter writer = new BufferedWriter(new FileWriter(file, true));
                writer.append("Time,Action,Target");
                writer.newLine();
                writer.flush();
                writer.close();
            } catch (IOException ex) {
                StringWriter stringWriter = new StringWriter();
                PrintWriter printWriter = new PrintWriter(stringWriter, true);
                ex.printStackTrace(printWriter);
                Log.e("Controller.Log", stringWriter.toString());
            }
        }

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(file, true));
            writer.append(logText);
            writer.newLine();
            writer.flush();
            writer.close();

        } catch(IOException ex) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter, true);
            ex.printStackTrace(printWriter);
            Log.e("Controller.Log", stringWriter.toString());
        }
    }


    /**
     * **************************************
     * ** MODEL METHODS BELOW THIS COMMENT **
     * **************************************
     */

    /**
     * @return a copy of the current model
     */
    public static ArrayList<Robot> getModel() {
        ArrayList<Robot> copyOfTheModel = new ArrayList<>();
        theModelLock.lock();
        for (Robot bot : theModel) {
            copyOfTheModel.add((Robot)bot.clone());
        }
        theModelLock.unlock();
        return copyOfTheModel;
    }

    /**
     * set the model
     * @param newModel is what the model should be now
     */
    private void setModel(ArrayList<Robot> newModel) {
        theModelLock.lock();
        theModel.clear();
        for (Robot bot : newModel) {
            theModel.add((Robot)bot.clone());
        }
        theModelLock.unlock();
    }
}

