import argparse
import os

import cv2


def wait_for_step_key() -> bool:
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key in (ord("l"), ord("L")):
            return True
        if key in (ord("q"), 27):
            return False

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--data-folder", type=str, default="capture/default", help="Path to the folder containing the captured data.")
    
    args = parser.parse_args()
    
    file_path = os.path.join(args.data_folder, "comparison.mp4")
    
    if not os.path.exists(file_path):
        print(f"File {file_path} does not exist.")
        return

    cap = cv2.VideoCapture(file_path)
    if not cap.isOpened():
        print(f"Failed to open video: {file_path}")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Reached end of video.")
                return

            cv2.imshow("Replay Comparison", frame)
            if not wait_for_step_key():
                return
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
