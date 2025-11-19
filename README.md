# 3dpSwerveDrive
Code repository utilized in running the 3d printed Swerve Drive! Check out this github repo for build documentation: [repo](https://github.com/Jtsaka/Robot_portfolio/blob/main/README.md)
The Swerve Drive utilizes a Modern Robotics 12V DC motor for drive, as well as a 35kg Continuous Servo for rotation of the module.

### Breakdown
A separate Python script on the edge computing device (RaspberryPi 5 or NVIDIA Jetson) runs the computer vision model, YOLO model v8, which detects an object (bottle) and calculates the distance and angle to the object. This information is then passed via UART to the Arduino, which then adjusts the wheel angle and distance to the object.

Another feature is that utilizing OpenAI's Faster Whisper model, a different Python script on the edgecomputing device takes in audio input, processes and transcribes it, and similarly sends the data via UART to the Arduino for processing to determine the angle and distance it should drive to.

### Object Oriented Programing (OOP) structure:
- Motors
- Continuous Servos
- Encoders
