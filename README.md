the main files are GCS PYTHON and SAVE , test is the control's embedded system and test2 is the drone's embedded system . 

Recently added and experimented with Madgwick filter (imu.ccp file) , and the results came up pretty smooth , there's still a yaw derivation ,will fix that after the 1st fly test

surely will separte the tasks of everything and pu each task on a different loop with different frequencies (for example the PID loop at 4ms, the radio communication at 100HZ, and the telemetry printing at 500HZ) to optimize the performance of the system.

That is Diagram which resumes the system architecture behind that work :

<img width="1821" height="1301" alt="controll diagram" src="https://github.com/user-attachments/assets/2307bf07-ccb1-4f19-939a-b15690fba4cd" />

