import sys
import yaml
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QFrame, QLabel, QVBoxLayout, QWidget,
    QSplitter, QPushButton
)
from PyQt5.QtCore import Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

from scipy.spatial import cKDTree

font1 = {'family': 'DejaVu Sans', 'color': 'black', 'size': 20}

# =========================
# ICP + ATE UTILITIES
# =========================
def best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A

    return R, t


def icp_full(A, B, max_iterations=20):
    A = A.copy()

    R_total = np.eye(2)
    t_total = np.zeros(2)

    for _ in range(max_iterations):
        tree = cKDTree(B)
        _, indices = tree.query(A)

        matched_B = B[indices]

        R, t = best_fit_transform(A, matched_B)

        A = (R @ A.T).T + t

        R_total = R @ R_total
        t_total = R @ t_total + t

    return R_total, t_total

def interpret_icp(R, t):
    # --- rotation (2D) → angle ---
    angle_rad = np.arctan2(R[1, 0], R[0, 0])
    angle_deg = np.degrees(angle_rad)

    # --- translation ---
    tx, ty = t

    return angle_deg, tx, ty

def symmetric_ate(gt_points, slam_points):
    if len(gt_points) == 0 or len(slam_points) == 0:
        return None

    tree_gt = cKDTree(gt_points)
    tree_slam = cKDTree(slam_points)

    d1, _ = tree_gt.query(slam_points)
    d2, _ = tree_slam.query(gt_points)

    return np.sqrt((np.mean(d1**2) + np.mean(d2**2)) / 2)


# =========================
# Matplotlib Canvas
# =========================
class MplCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)


# =========================
# Map Panel
# =========================
class MapPanel(QWidget):
    def __init__(self, title="Map"):
        super().__init__()

        self.title = title
        self.canvas = MplCanvas()
        self.toolbar = NavigationToolbar(self.canvas, self)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.toolbar)
        self.setLayout(layout)

    def plot_track(self, file_path):
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)

        track = data["track"]

        # -------- Robust loading --------
        left = track.get("left", [])
        right = track.get("right", [])
        unknown = track.get("unknown", [])

        # ---- classify unknown ----
        unknown_blue = [c for c in unknown if c.get("class") == "blue"]
        unknown_yellow = [c for c in unknown if c.get("class") == "yellow"]
        unknown_orange = [c for c in unknown if c.get("class") == "big-orange"]
        unknown_sorange = [c for c in unknown if c.get("class") == "small-orange"]

        # ---- classify known (left/right may contain mixed classes) ----
        def split_cones(cones):
            blue, yellow, orange, sorange = [], [], [], []
            for c in cones:
                cls = str(c.get("class"))
                if cls == "blue":
                    blue.append(c)
                elif cls == "yellow":
                    yellow.append(c)
                elif cls == "big-orange":
                    orange.append(c)
                elif cls == "small-orange":
                    sorange.append(c)  # treat small orange as big for now
            return blue, yellow, orange, sorange

        left_blue, left_yellow, left_orange, left_sorange = split_cones(left)
        right_blue, right_yellow, right_orange, right_sorange = split_cones(right)

        # ---- merge all ----
        blue = left_blue + right_blue + unknown_blue
        yellow = left_yellow + right_yellow + unknown_yellow
        orange = left_orange + right_orange + unknown_orange
        sorange = left_sorange + right_sorange + unknown_sorange

        # ---- extract positions ----
        def extract_xy(cones):
            x = [p["position"][0] for p in cones]
            y = [p["position"][1] for p in cones]
            return x, y

        blue_x, blue_y = extract_xy(blue)
        yellow_x, yellow_y = extract_xy(yellow)
        orange_x, orange_y = extract_xy(orange)
        sorange_x, sorange_y = extract_xy(sorange)

        # ---- store (ATE uses blue/yellow only) ----
        self.left_points = np.column_stack((blue_x, blue_y)) if blue_x else np.empty((0, 2))
        self.right_points = np.column_stack((yellow_x, yellow_y)) if yellow_x else np.empty((0, 2))
        self.orange_points = np.column_stack((orange_x, orange_y)) if orange_x else np.empty((0, 2))
        self.sorange_points = np.column_stack((sorange_x, sorange_y)) if sorange_x else np.empty((0, 2))


        # ---- plot ----
        self.canvas.ax.clear()

        if blue_x:
            self.canvas.ax.scatter(blue_x, blue_y, c="blue", s=10, label="Blue cones")
        if yellow_x:
            self.canvas.ax.scatter(yellow_x, yellow_y, c="orange", s=10, label="Yellow cones")
        if orange_x:
            self.canvas.ax.scatter(orange_x, orange_y, c="red", s=15, label="Big Orange cones")
        if sorange_x:
            self.canvas.ax.scatter(sorange_x, sorange_y, c="green", s=5, label="Orange cones")

        self.canvas.ax.set_title(self.title, fontdict=font1)
        self.canvas.ax.set_aspect("equal")
        self.canvas.ax.grid()
        self.canvas.ax.legend(bbox_to_anchor=(0.48, -0.1), loc="upper center", ncol=2)

        self.canvas.draw()

    def clear(self):
        self.canvas.ax.clear()
        self.canvas.draw()


# =========================
# Overlay Panel
# =========================
class OverlayPanel(QWidget):
    def __init__(self):
        super().__init__()

        self.canvas = MplCanvas()
        self.toolbar = NavigationToolbar(self.canvas, self)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.toolbar)
        self.setLayout(layout)

    def plot_overlay(self, gt_left, gt_right, gt_orange, gt_sorange,
                 slam_left, slam_right, slam_orange, slam_sorange):

        ax = self.canvas.ax
        ax.clear()

        # --- GT ---
        ax.scatter(gt_left[:, 0], gt_left[:, 1], s=20, label="GT Blue", edgecolors="blue", facecolors="none",)
        ax.scatter(gt_right[:, 0], gt_right[:, 1], s=20, label="GT Yellow", edgecolors="orange", facecolors="none")

        if len(gt_orange) > 0:
            ax.scatter(gt_orange[:, 0], gt_orange[:, 1],
                    s=50, label="GT Orange", edgecolors="red", facecolors="none")
            
        if len(gt_sorange) > 0:
            ax.scatter(gt_sorange[:, 0], gt_sorange[:, 1],
                    s=30, label="GT Small Orange", edgecolors="green", facecolors="none")

        # --- SLAM ---
        ax.scatter(slam_left[:, 0], slam_left[:, 1], c="blue", s=10, label="SLAM Blue")
        ax.scatter(slam_right[:, 0], slam_right[:, 1], c="orange", s=10, label="SLAM Yellow")

        if len(slam_orange) > 0:
            ax.scatter(slam_orange[:, 0], slam_orange[:, 1],
                    c="red", s=40, label="SLAM Big Orange")
            
        if len(slam_sorange) > 0:
            ax.scatter(slam_sorange[:, 0], slam_sorange[:, 1],
                    c="green", s=5, label="SLAM Orange")

        # --- Correspondences (UNCHANGED) ---
        tree_blue = cKDTree(gt_left)
        _, idx_blue = tree_blue.query(slam_left)

        for i, j in enumerate(idx_blue):
            ax.plot([slam_left[i][0], gt_left[j][0]],
                    [slam_left[i][1], gt_left[j][1]],
                    'gray', linewidth=0.5)

        tree_yellow = cKDTree(gt_right)
        _, idx_yellow = tree_yellow.query(slam_right)

        for i, j in enumerate(idx_yellow):
            ax.plot([slam_right[i][0], gt_right[j][0]],
                    [slam_right[i][1], gt_right[j][1]],
                    'gray', linewidth=0.5)

        ax.set_title("Overlay + Correspondences", fontdict=font1)
        ax.set_aspect("equal")
        ax.grid()
        ax.legend(bbox_to_anchor=(0.48, -0.1), loc="upper center", ncol=2)

        self.canvas.draw()

    def clear(self):
        self.canvas.ax.clear()
        self.canvas.draw()


# =========================
# Drop Panel
# =========================
class DropMapPanel(QWidget):
    def __init__(self, title, hint_text, color):
        super().__init__()

        self.map = MapPanel(title)

        self.label = QLabel(hint_text)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet(f"""
            QLabel {{
                border: 3px dashed {color};
                font-size: 18px;
                padding: 30px;
            }}
        """)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.map)

        self.setLayout(layout)
        self.setAcceptDrops(True)

        self.map.hide()

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        file_path = event.mimeData().urls()[0].toLocalFile()

        try:
            self.map.plot_track(file_path)
            self.label.hide()
            self.map.show()

            if hasattr(self, "controls"):
                self.controls.compute_ate()

        except Exception as e:
            self.label.setText(f"Error:\n{str(e)}")

    def reset(self):
        self.map.clear()
        self.map.hide()
        self.label.show()


# =========================
# Metric Box
# =========================
class MetricBox(QLabel):
    def __init__(self, title, color):
        super().__init__()

        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"""
            QLabel {{
                border: 2px solid {color};
                border-radius: 10px;
                padding: 15px;
                font-size: 16px;
                background-color: #f9f9f9;
            }}
        """)

        self.setText(f"{title}\n---")

    def set_value(self, title, value):
        self.setText(f"{title}\n{value:.3f} m")


# =========================
# Control Panel
# =========================
class ControlPanel(QWidget):
    def __init__(self, left, right, overlay):
        super().__init__()

        self.ate_label = QLabel("ATE Metrics")
        self.icp_label = QLabel("ICP Transform")

        self.ate_label.setAlignment(Qt.AlignCenter)
        self.icp_label.setAlignment(Qt.AlignCenter)

        self.ate_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                font-size: 16px;
                margin-top: 10px;
            }
        """)

        self.icp_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                font-size: 16px;
                margin-top: 20px;
            }
        """)

        self.separator = QFrame()
        self.separator.setFrameShape(QFrame.HLine)
        self.separator.setFrameShadow(QFrame.Sunken)
        self.separator.setStyleSheet("color: #888; margin-top: 10px; margin-bottom: 10px;")

        self.left = left
        self.right = right
        self.overlay = overlay

        layout = QVBoxLayout()  

        #ATE metric boxes
        self.blue_box = MetricBox("Blue ATE", "#2b6cb0")
        self.yellow_box = MetricBox("Yellow ATE", "#d69e2e")
        self.total_box = MetricBox("Total ATE", "#444")

        # ICP metric boxes
        self.rot_box = MetricBox("ICP Rotation", "#805ad5")
        self.trans_box = MetricBox("ICP Translation", "#2f855a")

        btn_left = QPushButton("Reset GT")
        btn_right = QPushButton("Reset SLAM")
        btn_all = QPushButton("Reset BOTH")

        btn_left.clicked.connect(self.left.reset)
        btn_right.clicked.connect(self.right.reset)
        btn_all.clicked.connect(self.reset_all)

        # --- ATE SECTION ---
        layout.addWidget(self.ate_label)
        layout.addWidget(self.blue_box)
        layout.addWidget(self.yellow_box)
        layout.addWidget(self.total_box)

        # --- separator ---
        layout.addWidget(self.separator)

        # --- ICP SECTION ---
        layout.addWidget(self.icp_label)
        layout.addWidget(self.rot_box)
        layout.addWidget(self.trans_box)

        layout.addSpacing(20)

        layout.addWidget(btn_left)
        layout.addWidget(btn_right)
        layout.addWidget(btn_all)
        layout.addStretch()

        self.setLayout(layout)

    def compute_ate(self):
        if not hasattr(self.left.map, "left_points") or not hasattr(self.right.map, "left_points"):
            return

        gt_left = self.left.map.left_points
        gt_right = self.left.map.right_points
        gt_orange = getattr(self.left.map, "orange_points", np.empty((0, 2)))
        gt_sorange = getattr(self.left.map, "sorange_points", np.empty((0, 2)))

        slam_left = self.right.map.left_points
        slam_right = self.right.map.right_points
        slam_orange = getattr(self.right.map, "orange_points", np.empty((0, 2)))
        slam_sorange = getattr(self.right.map, "sorange_points", np.empty((0, 2)))


        # --- ICP ONLY on blue + yellow ---
        gt_all = np.vstack((gt_left, gt_right))
        slam_all = np.vstack((slam_left, slam_right))

        R, t = icp_full(slam_all, gt_all)

        # --- align everything ---
        slam_left_aligned = (R @ slam_left.T).T + t
        slam_right_aligned = (R @ slam_right.T).T + t
        slam_orange_aligned = (R @ slam_orange.T).T + t if len(slam_orange) else slam_orange
        slam_sorange_aligned = (R @ slam_sorange.T).T + t if len(slam_sorange) else slam_sorange

        # --- ATE ---
        blue_ate = symmetric_ate(gt_left, slam_left_aligned)
        yellow_ate = symmetric_ate(gt_right, slam_right_aligned)
        total_ate = (blue_ate + yellow_ate) / 2

        self.blue_box.set_value("Blue ATE", blue_ate)
        self.yellow_box.set_value("Yellow ATE", yellow_ate)
        self.total_box.set_value("Total ATE", total_ate)

        # --- ICP metrics ---
        angle_deg, tx, ty = interpret_icp(R, t)

        self.rot_box.setText(f"ICP Rotation\n{angle_deg:.2f}°")
        self.trans_box.setText(
            f"ICP Translation\n"
            f"dx = {tx:.3f} m\n"
            f"dy = {ty:.3f} m"
        )

        # --- UPDATED overlay call ---
        self.overlay.plot_overlay(
            gt_left, gt_right, gt_orange, gt_sorange,
            slam_left_aligned, slam_right_aligned, slam_orange_aligned, slam_sorange_aligned    
        )

    def reset_all(self):
        self.left.reset()
        self.right.reset()
        self.overlay.clear()

        self.blue_box.setText("Blue ATE\n---")
        self.yellow_box.setText("Yellow ATE\n---")
        self.total_box.setText("Total ATE\n---")


# =========================
# Main Window
# =========================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Track Viewer")
        self.resize(1500, 700)

        self.left_panel = DropMapPanel("GROUND TRUTH", "Drop Ground Truth map", "#2b6cb0")
        self.right_panel = DropMapPanel("SLAM MAP", "Drop SLAM map", "#c53030")
        self.overlay_panel = OverlayPanel()

        self.controls = ControlPanel(self.left_panel, self.right_panel, self.overlay_panel)

        self.left_panel.controls = self.controls
        self.right_panel.controls = self.controls

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.left_panel)
        splitter.addWidget(self.right_panel)
        splitter.addWidget(self.overlay_panel)
        splitter.addWidget(self.controls)

        splitter.setSizes([400, 400, 500, 200])

        layout = QVBoxLayout()
        layout.addWidget(splitter)
        self.setLayout(layout)


# =========================
# Run
# =========================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())