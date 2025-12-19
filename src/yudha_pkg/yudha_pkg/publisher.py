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
        cv2.createTrackbar("S Min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("S Max", "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("V Min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("V Max", "HSV Calibration", 255, 255, self.nothing)

        self.timer = self.create_timer(0.033, self.process_and_publish)
        
        self.get_logger().info("Node Detektor Multi-Warna ROS2 dimulai.")

    def get_hsv_range(self, color):
        """Helper untuk mendapatkan parameter filter warna standar."""
        if color == "red":
            lower1 = np.array([0, 80, 80])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([170, 80, 80])
            upper2 = np.array([179, 255, 255])
            return (lower1, upper1), (lower2, upper2)
        elif color == "green":
            return (np.array([35, 80, 80]), np.array([85, 255, 255]))
        elif color == "blue":
            return (np.array([90, 80, 80]), np.array([130, 255, 255]))
        return None

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

        final_info = "Tidak Ada Objek"
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > 1000:
                x, y, w, h = cv2.boundingRect(largest)
                roi_hsv = hsv[y:y+h, x:x+w]

                r_range1, r_range2 = self.get_hsv_range("red")
                m_red = cv2.bitwise_or(cv2.inRange(roi_hsv, r_range1[0], r_range1[1]),
                                      cv2.inRange(roi_hsv, r_range2[0], r_range2[1]))
                
                g_range = self.get_hsv_range("green")
                m_green = cv2.inRange(roi_hsv, g_range[0], g_range[1])
                
                b_range = self.get_hsv_range("blue")
                m_blue = cv2.inRange(roi_hsv, b_range[0], b_range[1])

                counts = {
                    "RED": cv2.countNonZero(m_red),
                    "GREEN": cv2.countNonZero(m_green),
                    "BLUE": cv2.countNonZero(m_blue)
                }
                
                color_label = max(counts, key=counts.get)
                if counts[color_label] == 0: color_label = "Unknown"

                peri = cv2.arcLength(largest, True)
                approx = cv2.approxPolyDP(largest, 0.02 * peri, True)

                if len(approx) == 3:
                    shape = "Segitiga"
                elif len(approx) == 4:
                    shape = "Persegi"
                elif len(approx) > 6:
                    shape = "Lingkaran"
                else:
                    shape = "Bentuk Tidak Beraturan"

                final_info = f"{color_label} {shape}"

                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, final_info, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        msg = String()
        msg.data = f"Terdeteksi: {final_info}"
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
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