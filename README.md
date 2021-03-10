# Balancing-robot

This is an arduino script that balances a two-wheeled robot using a PD controller and an MPU6050 accelerometer/gyroscope.

The angle the robot is at is obtained through fusing measurements from the accelerometer and gyroscope together for better accuracy, and is done through the "MPU6050 light" library.

Board used in my experiments is the Elegoo Mega 2560. Motors are Dagu DG01D (though if I could throw more money at it I would improve the motors - they don't have a very fast top speed/torque which makes it harder to recover from more severe angles)

This code combined with the hardware described allows the robot to balance, sometimes with slight deviation and settling time around the centrepoint. Again this would be improved with better motors - I have to give a large gain to try and get the balance from larger angles, and the large gain means that at smaller angles too much power is given and it seesaws sometimes.
