mingw32-make -j4
mingw32-make rebuild
mingw32-make "-j$env:NUMBER_OF_PROCESSORS"
全核编译
x向前roll顺时针负左转正
y轴向右pitch顺时针正抬头正
Z向上yaw顺时针正左翻正
yaw = IMU_data->Yaw * DEGREE_2_RAD;
    yaw_dot = IMU_data->Gyro[2];
    pitch = -IMU_data->Pitch * DEGREE_2_RAD;
    pitch_dot = -IMU_data->Gyro[0];
    roll = IMU_data->Roll * DEGREE_2_RAD;
    roll_dot = IMU_data->Gyro[1];