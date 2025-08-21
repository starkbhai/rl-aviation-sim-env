from camera_sub_module import AircraftCameraSubscriber
import time
import cv2  # OpenCV for displaying images

def main():
    # Initialize the subscriber
    camera_sub = AircraftCameraSubscriber()

    print("✅ Camera Subscriber Initialized")

    while True:
        # Get the latest image frame
        frame = camera_sub.get_latest_image()

        if frame is not None:
            # Show the live image feed
            cv2.imshow("Live Camera Feed", frame)

            # Print image info
            info = camera_sub.get_image_info()
            print(f"🖼️ Image Info: {info}")

            # Show frame count
            count = camera_sub.get_frame_count()
            print(f"📸 Frame Count: {count}")

            # Check if frames are being received
            is_live = camera_sub.is_receiving_frames()
            print(f"🔄 Live Feed Status: {'Receiving' if is_live else 'Stale'}")

            # Show last update time
            last_update = camera_sub.get_last_update_time()
            print(f"⏱️ Last Update: {last_update}")

            # Show current FPS
            fps = camera_sub.get_frame_rate()
            print(f"🎞️ Frame Rate: {fps:.2f} FPS")

            print(f"🧾 Frame Data (shape: {frame.shape}, dtype: {frame.dtype}):")
            print(frame if frame.size < 500 else frame[:5, :5])  # Truncate if too large

        else:
            print("⚠️ No frame received")

        # Break loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.5)  # Adjust refresh rate as needed

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
