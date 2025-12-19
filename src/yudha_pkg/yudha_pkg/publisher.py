import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class ImageDetectorNode(Node):
    def __init__(self):
        super().__init__("image_detector_node")
        
        self.publisher_ = self.create_publisher(String, "yudha_chatter", 10)
        
        self.cap = cv2.VideoCapture(0)
        
        cv2.namedWindow("HSV Calibration")
        cv2.createTrackbar("H Min", "HSV Calibration", 0, 179, self.nothing)
        cv2.createTrackbar("H Max", "HSV Calibration", 179, 179, self.nothing)
        cv2.createTrackbar("S Min", "HSV Calibration", 0, 255, self.nothing)
        cv2.createTrackbar("S Max", "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("V Min", "HSV Calibration", 0, 255, self.nothing)
        cv2.createTrackbar("V Max", "HSV Calibration", 255, 255, self.nothing)

        self.timer = self.create_timer(0.033, self.process_and_publish)
        
        self.get_logger().info("Node Detektor dimulai. Publikasi setiap 1 detik.")

    def nothing(self, x):
        pass

    def process_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Gagal mengambil gambar dari kamera.")
            return

        frame = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H Min", "HSV Calibration")
        h_max = cv2.getTrackbarPos("H Max", "HSV Calibration")
        s_min = cv2.getTrackbarPos("S Min", "HSV Calibration")
        s_max = cv2.getTrackbarPos("S Max", "HSV Calibration")
        v_min = cv2.getTrackbarPos("V Min", "HSV Calibration")
        v_max = cv2.getTrackbarPos("V Max", "HSV Calibration")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_shape = "Tidak Ada Objek"
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > 500:
                peri = cv2.arcLength(largest, True)
                approx = cv2.approxPolyDP(largest, 0.02 * peri, True)

                if len(approx) == 3:
                    detected_shape = "Segitiga"
                elif len(approx) == 4:
                    detected_shape = "Persegi"
                elif len(approx) > 6:
                    detected_shape = "Lingkaran / Bola"
                else:
                    detected_shape = "Bentuk Tidak Beraturan"

                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, f"{detected_shape}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        msg = String()
        msg.data = f"Terdeteksi: {detected_shape}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()