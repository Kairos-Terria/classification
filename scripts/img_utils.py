import cv2
import numpy as np

class ColorExtractor:
    def __init__(self):
        self.cap = cv2.VideoCapture(2)

        self.threshold = {'red': [np.array([0, 0, 100]), np.array([120, 120, 255])], #red
                          'yellow': [np.array([0, 100, 100]), np.array([100, 230, 255])], #yellow
                          'blue': [np.array([100, 140, 200]), np.array([255, 255, 200])], #blue
                          'purple': [np.array([250, 30, 30]), np.array([300, 100, 100])]} #purple

        self.threshold_len = len(self.threshold)

        self.color = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.threshold]

        self.width = 640
        self.height = 480

    def get_color(self):

        def draw_bounding_box(contours, color):
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                min_size = 80
                if w >= min_size and h >= min_size:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = list()
            for k, v in self.threshold.items():
                mask.append(cv2.inRange(hsv, v[0], v[1]))

            contours = list()
            for i in range(self.threshold_len):
                contours.append(cv2.findContours(mask[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))

            for i, (k, _) in enumerate(self.threshold.items()):
                if len(contours[i][0]) > 0:
                    print(k,':', len(contours[i][0]))
                else:
                    print(k,':', 'no')

            cv2.namedWindow("window", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("window", width=self.width, height=self.height)
            cv2.imshow("window", frame)
            cv2.waitKey(1)

if __name__=='__main__':
    c = ColorExtractor()
    c.get_color()
