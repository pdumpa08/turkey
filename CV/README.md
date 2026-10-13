# Automatic Turkish Ice Cream Man CV

We built a computer vision model that processes video inputs from a HD webcam using OpenCV and then processes that video stream with Google's MediaPipe library, allowing us to detect and track hands in each of the frames. This information is then sent along to the firmware to control the motors.

## What It Detects

The program uses a webcam to detect and track one human hand. It estimates:

- The hand's palm center
- The hand's movement direction and velocity
- Whether the hand is approaching the rod
- Whether the hand's projected path comes close enough to be considered a threat
- A perpendicular dodge direction that moves the hand farther from the rod

The video output displays (not in final product, but useful debug tool):

- Green dot: detected palm center
- Blue arrow: estimated hand velocity
- Red arrow: calculated dodge direction
- Yellow dot: rod position

## Model

This project uses Google's MediaPipe Hands model through the Python `mediapipe` package.

MediaPipe Hands was chosen because it:

- Works with ordinary webcams and runs locally
- Provides enough landmark information to estimate the palm position and movement
- Is lightweight enough for real-time interaction

The program uses landmarks 0, 5, and 17 to calculate the palm center. I've had experience with YOLO before and it works great, but I ruled it out for this particular use-case because the models were often very compute-intensive and MediaPipe had a strong hand recognition model.

## How It Works

1. MediaPipe Hands detects the hand landmarks.
2. The program calculates the palm center from three hand landmarks. (custom analytics form here on)
3. The current palm center is compared with the previous frame to estimate velocity.
4. Velocity is smoothed using an exponential moving average.
5. The program checks whether the hand is moving toward the rod.
6. It calculates the perpendicular distance from the hand's movement path to the rod.
7. If the path is close enough, the program tests both perpendicular dodge directions.
8. The direction that places the hand farther from the rod is selected.
9. The camera frame displays the detection and direction information.

The threat detection uses the hand's velocity vector and the fixed rod position. A hand is considered a threat when:

It is moving toward the rod AND Its movement path passes within `THREAT_DIST_PX` pixels of the rod.