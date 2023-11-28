package com.i8r441m.vio;

import android.Manifest;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.Image;
import android.os.Bundle;
import android.os.Environment;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import com.google.ar.core.Camera;
import com.google.ar.core.CameraConfig;
import com.google.ar.core.CameraConfigFilter;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.sceneform.FrameTime;
import com.google.ar.sceneform.ux.ArFragment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.text.SimpleDateFormat;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
public class ARActivity extends AppCompatActivity implements SensorEventListener {
    private ArFragment arFragment;
    private SensorManager sensorManager;
    private Button recordButton;
    private Button saveButton;
    private float[] accelData = new float[3];
    private float[] gyroData = new float[3];
    private boolean isAccelDataAvailable = false;
    private boolean isGyroDataAvailable = false;
    private float[] pose_translational;
    private float[] pose_rotational;
    private boolean recording = false;
    private ExecutorService backgroundExecutor;
    private OutputStream imuDataStream;
    private OutputStream poseDataStream;
    private OutputStream imgDataStream;
    private boolean imuDataStreamOpen = false;
    private boolean poseDataStreamOpen = false;
    private boolean imgDataStreamOpen = false;
    private String imagesPath;
    private long imageCounter = 0;
    private TextView recordingState;
    private Sensor accelerometer;
    private Sensor gyroscope;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_ar);

        backgroundExecutor = Executors.newFixedThreadPool(6);

        recordButton = findViewById(R.id.record_button);
        saveButton = findViewById(R.id.save_button);

        arFragment = (ArFragment) getSupportFragmentManager().findFragmentById(R.id.ar_fragment);

        arFragment.getArSceneView().getScene().addOnUpdateListener(this::onUpdateFrame);
        arFragment.getPlaneDiscoveryController().hide();
        arFragment.getPlaneDiscoveryController().setInstructionView(null);
        arFragment.getArSceneView().getPlaneRenderer().setEnabled(false);

        recordButton.setVisibility(View.VISIBLE);

        recordButton.setOnClickListener(v -> record());
        saveButton.setOnClickListener(v -> save());

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_FASTEST);

        recordingState = findViewById(R.id.recording);

    }
    @Override
    protected void onResume() {
        super.onResume();
        if (arFragment.getArSceneView().getSession() != null) {
            arFragment.getArSceneView().getSession().pause();
            CameraConfigFilter filter = new CameraConfigFilter(arFragment.getArSceneView().getSession());
            List<CameraConfig> cameraConfigs = arFragment.getArSceneView().getSession().getSupportedCameraConfigs(filter);
            CameraConfig highestResConfig = Collections.max(cameraConfigs,
                    Comparator.comparingInt(c -> c.getImageSize().getWidth() * c.getImageSize().getHeight()));
            arFragment.getArSceneView().getSession().setCameraConfig(highestResConfig);

            try {
                arFragment.getArSceneView().getSession().resume();
            } catch (CameraNotAvailableException e) {
                throw new RuntimeException(e);
            }
        }
    }
    private void record() {
        if (!ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 1);
        }

        arFragment.getArSceneView().getSession().pause();
        CameraConfigFilter filter = new CameraConfigFilter(arFragment.getArSceneView().getSession());
        List<CameraConfig> cameraConfigs = arFragment.getArSceneView().getSession().getSupportedCameraConfigs(filter);
        CameraConfig highestResConfig = Collections.max(cameraConfigs,
                Comparator.comparingInt(c -> c.getImageSize().getWidth() * c.getImageSize().getHeight()));
        arFragment.getArSceneView().getSession().setCameraConfig(highestResConfig);

        try {
            arFragment.getArSceneView().getSession().resume();
        } catch (CameraNotAvailableException e) {
            throw new RuntimeException(e);
        }
        
        long epochTime = System.currentTimeMillis() / 1000;
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
        String formattedTime = sdf.format(new Date(epochTime * 1000));

        String baseDirectory = getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS).getAbsolutePath() + "/" + formattedTime;
        imagesPath = baseDirectory + "/images";
        Path imuDataPath = Paths.get(baseDirectory + "/imu_data.csv");
        Path poseDataPath = Paths.get(baseDirectory + "/pose_data.csv");
        Path imgDataPath = Paths.get( baseDirectory + "/img_data.csv");

        createDirectory(baseDirectory);
        createDirectory(imagesPath);

        try {
            imuDataStream = Files.newOutputStream(imuDataPath, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            imuDataStreamOpen = true;
            // TODO: Log file created
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try {
            poseDataStream = Files.newOutputStream(poseDataPath, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            poseDataStreamOpen = true;
            // TODO: Log file created
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try {
            imgDataStream = Files.newOutputStream(imgDataPath, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            imgDataStreamOpen = true;
            // TODO: Log file created
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        if (imuDataStreamOpen && poseDataStreamOpen && imgDataStreamOpen) {
            recordButton.setVisibility(View.GONE);
            saveButton.setVisibility(View.VISIBLE);
            recordingState.setVisibility(View.VISIBLE);
            recording = true;
            // TODO: Log status
        }

    }

    private void createDirectory(String directoryPath) {
        File directory = new File(directoryPath);
        if (!directory.exists()) {
            if (directory.mkdirs()) {
                // TODO: handle this
            } else {
                // TODO: handle this
            }
        }
    }

    private void updateState(Camera camera) {
        TextView trackingState = findViewById(R.id.status);
        trackingState.setText(camera.getTrackingState().toString());
        trackingState.setTextColor(camera.getTrackingState() == TrackingState.TRACKING ? 0xFF00FF00 : 0xFFFF0000);
        // TODO: Log state change
    }

    private void onUpdateFrame(FrameTime frameTime) {
        Frame frame = arFragment.getArSceneView().getArFrame();
        if (frame != null) {
            long timestamp = frame.getTimestamp();
            Camera camera = frame.getCamera();
            updateState(camera);
            Pose pose = camera.getDisplayOrientedPose();
            pose_translational = pose.getTranslation();
            pose_rotational = pose.getRotationQuaternion();
            if (recording) {
                backgroundExecutor.submit(() -> writePoseData(timestamp, pose_translational, pose_rotational));
                backgroundExecutor.submit(() -> {
                    try {
                        Image img = frame.acquireCameraImage();
                        int imageFormat = img.getFormat();
                        int height = img.getHeight();
                        int width = img.getWidth();
                        String filename = String.valueOf(imageCounter++) + ".yuv";
                        backgroundExecutor.submit(() -> saveImage(img, filename));
                        writeImgData(timestamp, filename, imageFormat, height, width);
                    } catch (NotYetAvailableException e) {
                        // TODO: log this (check, maybe already logged by default)
                    }
                });
            }
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, accelData, 0, event.values.length);
                isAccelDataAvailable = true;
                break;
            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(event.values, 0, gyroData, 0, event.values.length);
                isGyroDataAvailable = true;
                break;
        }

        if (isAccelDataAvailable && isGyroDataAvailable && recording) {
            long timestamp = event.timestamp;
            backgroundExecutor.submit(() -> writeImuData(timestamp, accelData, gyroData));
            isAccelDataAvailable = false;
            isGyroDataAvailable = false;
        }
    }

    private synchronized void saveImage(Image image, String filename) {
        File file = new File(imagesPath, filename);

        try (FileOutputStream outputStream = new FileOutputStream(file)) {
            ByteBuffer buffer;
            // TODO:
            for (Image.Plane plane : image.getPlanes()) {
                buffer = plane.getBuffer();
                byte[] bytes = new byte[buffer.capacity()];
                buffer.get(bytes);
                outputStream.write(bytes);
           }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            image.close();
        }
    }

    private synchronized void writeImgData(long timestamp, String filename, int imageFormat, int height, int width) {
        String line = String.format("%d,%s,%d,%d,%d%n",
                timestamp, filename, imageFormat, height, width);

        try {
            byte[] lineBytes = line.getBytes(StandardCharsets.UTF_8);
            imgDataStream.write(lineBytes);
            imgDataStream.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private synchronized void writePoseData(long timestamp, float[] pose_translational, float[] pose_rotational) {
        String line = String.format("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                timestamp,
                pose_translational[0], pose_translational[1], pose_translational[2],
                pose_rotational[0], pose_rotational[1], pose_rotational[2], pose_rotational[3]);

        try {
            byte[] lineBytes = line.getBytes(StandardCharsets.UTF_8);
            poseDataStream.write(lineBytes);
            poseDataStream.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private synchronized void writeImuData(long timestamp, float[] accelData, float[] gyroData) {
        String line = String.format("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                timestamp,
                accelData[0], accelData[1], accelData[2],
                gyroData[0], gyroData[1], gyroData[2]);

        try {
            byte[] lineBytes = line.getBytes(StandardCharsets.UTF_8);
            imuDataStream.write(lineBytes);
            imuDataStream.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // TODO: implement
    }

    private void save() {
        super.onPause();
        sensorManager.unregisterListener(this);
        arFragment.getArSceneView().pause();
        arFragment.getArSceneView().getSession().close();
        backgroundExecutor.shutdown();
        try {
            // TODO: Log this
            backgroundExecutor.awaitTermination(60, TimeUnit.SECONDS);
        } catch (InterruptedException ie) {
            backgroundExecutor.shutdownNow();
            Thread.currentThread().interrupt();
            // TODO: Log this
        }
        finish();
    }
}
