# AnalysisFrontLeg_Kinematics
To implement the Raspberry Pi 4 and Bosch BNO055 IMU sensor for orientation and motion analysis Analysis of Front Leg Kinematics of Cricket Bowler Using Wearable   hardwaere and python code
Analyzing the front leg kinematics of a cricket bowler using a Raspberry Pi 4 and Bosch BNO055 IMU sensor involves several steps, from setting up the hardware to developing the Python code for data collection and analysis. Here's a detailed guide:

**Hardware Setup:**

1. **Raspberry Pi Setup:**

   - Set up your Raspberry Pi 4 with a microSD card and a power supply.
   - Connect the Raspberry Pi to the internet.

2. **BNO055 IMU Setup:**

   - Connect the BNO055 IMU sensor to the Raspberry Pi using I2C communication (SDA and SCL pins).
   - Provide power (VCC) and ground (GND) connections to the sensor.

**Python Code Implementation:**

1. **Install Libraries:**

   Install the required libraries for communication with the BNO055 sensor and data analysis.

   ```bash
   pip install adafruit-circuitpython-bno055 numpy matplotlib
   ```

2. **Data Collection Script (`imu_data_collection.py`):**

   Write a Python script to collect sensor data from the BNO055 IMU sensor.

   ```python
   import time
   import board
   import busio
   import adafruit_bno055
   import numpy as np
   import matplotlib.pyplot as plt

   i2c = busio.I2C(board.SCL, board.SDA)
   sensor = adafruit_bno055.BNO055(i2c)

   data = {'timestamp': [], 'euler_angles': []}

   try:
       while True:
           timestamp = time.time()
           euler_angles = sensor.euler

           data['timestamp'].append(timestamp)
           data['euler_angles'].append(euler_angles)

           time.sleep(0.01)  # Adjust the sleep time as needed
   except KeyboardInterrupt:
       print("Data collection stopped")

   np.save('imu_data.npy', data)
   ```

3. **Data Analysis Script (`imu_data_analysis.py`):**

   Write a Python script to analyze and visualize the collected data.

   ```python
   import numpy as np
   import matplotlib.pyplot as plt

   data = np.load('imu_data.npy', allow_pickle=True).item()

   timestamps = data['timestamp']
   euler_angles = data['euler_angles']

   pitch_angles = np.array([euler[1] for euler in euler_angles])

   plt.plot(timestamps, pitch_angles)
   plt.xlabel('Time (s)')
   plt.ylabel('Pitch Angle (degrees)')
   plt.title('Front Leg Kinematics of Cricket Bowler')
   plt.show()
   ```

**Data Collection and Analysis:**

1. Run the `imu_data_collection.py` script on your Raspberry Pi to collect sensor data. Stop the script when you've collected sufficient data by pressing `Ctrl + C`.

2. Transfer the `imu_data.npy` file to your development machine for analysis.

3. Run the `imu_data_analysis.py` script on your development machine to visualize the front leg kinematics (pitch angles) of the cricket bowler over time.

**Fine-Tuning and Interpretation:**

1. Analyze the pitch angle data to understand the bowler's leg kinematics during different phases of the bowling action.
   
2. You might need to apply filters or signal processing techniques to enhance the accuracy of the kinematic analysis.

3. Compare the kinematic data with known patterns of efficient bowling techniques and identify areas for improvement.

Remember that this is a basic example and advanced analysis might involve calibrating the sensor, applying sensor fusion algorithms for accurate orientation, and using machine learning for phase classification. Collaborating with experts in biomechanics, sensor fusion, and data analysis can help refine your approach.
*Using MPU6050*
Analyzing the kinematics of a cricket player's motion using an MPU6050 sensor and a Raspberry Pi involves capturing motion data and interpreting it to understand the player's movements. Here's a step-by-step guide:

**Hardware Setup:**

1. **Raspberry Pi Setup:**
   - Set up your Raspberry Pi with a compatible operating system (Raspberry Pi OS).
   - Connect to the internet and ensure you have the necessary software libraries installed.

2. **MPU6050 Connection:**
   - Connect the MPU6050 sensor to the Raspberry Pi using I2C communication (SDA and SCL pins).
   - Supply power (VCC) and ground (GND) to the sensor.

**Python Code Implementation:**

1. **Install Libraries:**
   Install the necessary libraries for I2C communication and data analysis.

   ```bash
   pip install smbus2 numpy matplotlib
   ```

2. **Data Collection Script (`mpu6050_data_collection.py`):**
   Write a Python script to collect motion data from the MPU6050 sensor.

   ```python
   import time
   import smbus2
   import numpy as np
   import matplotlib.pyplot as plt

   bus = smbus2.SMBus(1)  # Use 1 for Raspberry Pi 2 or 3, use 0 for older models

   MPU6050_ADDR = 0x68

   def read_raw_data(addr):
       high = bus.read_byte_data(MPU6050_ADDR, addr)
       low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
       value = ((high << 8) | low)
       if value > 32768:
           value = value - 65536
       return value

   data = {'timestamp': [], 'acceleration': [], 'gyroscope': []}

   try:
       while True:
           timestamp = time.time()
           accel_x = read_raw_data(0x3B)
           accel_y = read_raw_data(0x3D)
           accel_z = read_raw_data(0x3F)

           gyro_x = read_raw_data(0x43)
           gyro_y = read_raw_data(0x45)
           gyro_z = read_raw_data(0x47)

           data['timestamp'].append(timestamp)
           data['acceleration'].append((accel_x, accel_y, accel_z))
           data['gyroscope'].append((gyro_x, gyro_y, gyro_z))

           time.sleep(0.01)  # Adjust the sleep time as needed
   except KeyboardInterrupt:
       print("Data collection stopped")

   np.save('mpu6050_data.npy', data)
   ```

3. **Data Analysis Script (`mpu6050_data_analysis.py`):**
   Write a Python script to analyze and visualize the collected data.

   ```python
   import numpy as np
   import matplotlib.pyplot as plt

   data = np.load('mpu6050_data.npy', allow_pickle=True).item()

   timestamps = data['timestamp']
   accelerations = np.array(data['acceleration'])
   gyroscopes = np.array(data['gyroscope'])

   plt.figure(figsize=(12, 6))
   plt.subplot(2, 1, 1)
   plt.plot(timestamps, accelerations)
   plt.xlabel('Time (s)')
   plt.ylabel('Acceleration (raw)')
   plt.title('Acceleration Data')

   plt.subplot(2, 1, 2)
   plt.plot(timestamps, gyroscopes)
   plt.xlabel('Time (s)')
   plt.ylabel('Gyroscope (raw)')
   plt.title('Gyroscope Data')

   plt.tight_layout()
   plt.show()
   ```

**Data Collection and Analysis:**

1. Run the `mpu6050_data_collection.py` script on your Raspberry Pi to collect motion data. Stop the script when you've collected sufficient data by pressing `Ctrl + C`.

2. Transfer the `mpu6050_data.npy` file to your development machine for analysis.

3. Run the `mpu6050_data_analysis.py` script on your development machine to visualize the acceleration and gyroscope data over time.

**Interpretation and Further Analysis:**

1. Analyze the raw acceleration and gyroscope data to understand the cricket player's motion patterns.

2. Apply signal processing techniques to filter and smooth the data for better analysis results.

3. Extract meaningful features from the raw data to calculate angles, angular velocities, and accelerations.

4. Compare the motion data to known patterns of efficient cricket movements to provide insights into the player's performance.

Remember that raw data from sensors like the MPU6050 might need additional calibration and processing to accurately represent real-world motions. This example provides a basic starting point, but more sophisticated analysis and interpretation techniques might be required for comprehensive biomechanical analysis.

Calculating kinematics using the MPU6050 sensor and Arduino involves similar steps as mentioned earlier. Here's a guide on how to implement kinematics calculations using the MPU6050 sensor with Arduino:

**1. Hardware Setup:**

Connect the MPU6050 sensor to the Arduino board using the I2C communication protocol. The sensor provides accelerometer and gyroscope data that you'll use for kinematics calculations.

**2. Arduino Code Implementation:**

Write an Arduino sketch to read data from the MPU6050 sensor, convert the raw data into physical units, and perform kinematics calculations.

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  mpu.setDMPEnabled(true);
}

void loop() {
  // Read accelerometer and gyroscope data
  int16_t accelX = mpu.getAccelerationX();
  int16_t accelY = mpu.getAccelerationY();
  int16_t accelZ = mpu.getAccelerationZ();
  
  int16_t gyroX = mpu.getRotationX();
  int16_t gyroY = mpu.getRotationY();
  int16_t gyroZ = mpu.getRotationZ();

  // Convert raw data to physical units
  float accelX_mss = accelX / 16384.0;
  float accelY_mss = accelY / 16384.0;
  float accelZ_mss = accelZ / 16384.0;

  float gyroX_dps = gyroX / 131.0;
  float gyroY_dps = gyroY / 131.0;
  float gyroZ_dps = gyroZ / 131.0;

  // Perform kinematics calculations (example: linear acceleration)
  float linearAccelX = accelX_mss - gravityX;
  float linearAccelY = accelY_mss - gravityY;
  float linearAccelZ = accelZ_mss - gravityZ;

  // Print the calculated kinematic parameters
  Serial.print("Linear Acceleration X: ");
  Serial.println(linearAccelX);
  
  delay(100);
}
```

In this example, the sketch reads raw accelerometer and gyroscope data from the MPU6050 sensor, converts the data to physical units, and calculates linear acceleration by subtracting the gravity component. You can extend this by integrating, differentiating, and applying kinematic equations based on your specific application.

**3. Visualization and Interpretation:**

Visualize and interpret the calculated kinematic parameters using the Arduino Serial Monitor or other visualization tools. You can also use serial communication to transmit data to a computer for more advanced analysis.

Remember that kinematics calculations might require careful handling of noise, calibration, and numerical errors. Advanced techniques such as sensor fusion algorithms could be applied to improve accuracy.
