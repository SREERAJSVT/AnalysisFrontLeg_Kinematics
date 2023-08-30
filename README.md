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
