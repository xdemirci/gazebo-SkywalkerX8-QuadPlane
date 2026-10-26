# aruco_test.py - ArUco 4x4_50 (+ opsiyonel QR), model yok, sadece klasik OpenCV
import argparse
import time

import cv2
import numpy as np

GST_UDP = (
    "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, "
    "clock-rate=(int)90000, encoding-name=(string)H264\" ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false"
)

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# OpenCV yeni (>=4.7) / eski aruco API uyumu
if hasattr(cv2.aruco, "ArucoDetector"):
    _det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(ARUCO_DICT), cv2.aruco.DetectorParameters()
    )

    def detect_aruco(gray):
        corners, ids, _ = _det.detectMarkers(gray)
        return corners, ids
else:
    _dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
    _par = cv2.aruco.DetectorParameters_create()

    def detect_aruco(gray):
        corners, ids, _ = cv2.aruco.detectMarkers(gray, _dict, parameters=_par)
        return corners, ids


def open_source(src: str):
    if src == "udp":                      # Gazebo / drone yayını
        return cv2.VideoCapture(GST_UDP, cv2.CAP_GSTREAMER)
    if src.isdigit():                     # webcam: 0, 1, ...
        cap = cv2.VideoCapture(int(src), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    return cv2.VideoCapture(src)          # dosya yolu veya RTSP adresi


ap = argparse.ArgumentParser()
ap.add_argument("--src", default="0", help="0 (webcam) | udp | dosya/rtsp")
ap.add_argument("--qr", action="store_true", help="QR tespitini de aç (pyzbar)")
ap.add_argument("--no-clahe", action="store_true", help="ArUco için CLAHE'yi kapat")
args = ap.parse_args()

if args.qr:
    from pyzbar import pyzbar

cap = open_source(args.src)
if not cap.isOpened():
    raise SystemExit(f"Kaynak açılamadı: {args.src}")

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
t_prev, fps = time.time(), 0.0



if __name__ == "__main__":
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # --- ArUco (H264 sıkıştırmalı yayında CLAHE yardımcı oluyor) ---
            g_aruco = gray if args.no_clahe else clahe.apply(gray)
            corners, ids = detect_aruco(g_aruco)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for c, i in zip(corners, ids.flatten()):
                    cx, cy = c[0].mean(axis=0)
                    cv2.circle(frame, (int(cx), int(cy)), 7, (0, 0, 255), -1)
                    cv2.putText(frame, f"ARUCO DETECTED! ID:{i} ({int(cx)},{int(cy)})", (int(cx) - 210, int(cy) - 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

                    # cv2.putText(frame, f"Aruco Tespit", (int(cx) + 10, int(cy) - 200),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # --- QR (adaptive threshold, CLAHE değil) ---
            if args.qr:
                bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY, 31, 5)
                for q in pyzbar.decode(bw):
                    pts = np.array([(p.x, p.y) for p in q.polygon], np.int32)
                    cv2.polylines(frame, [pts], True, (255, 0, 255), 2)
                    cv2.putText(frame, q.data.decode(errors="ignore")[:30], tuple(int(v) for v in pts[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now
            n = 0 if ids is None else len(ids)
            cv2.putText(frame, f"FPS {fps:.1f} | detected aruco count : {n}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)

            cv2.imshow("aruco", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nİptal edildi.")
        cap.release()
        cv2.destroyAllWindows()
        raise SystemExit(0)