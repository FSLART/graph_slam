import matplotlib.pyplot as plt

# Black/dark mode for Matplotlib figures (readable on dark terminals)
plt.style.use("dark_background")
plt.rcParams.update(
    {
        "figure.facecolor": "#000000",
        "axes.facecolor": "#000000",
        "savefig.facecolor": "#000000",
        "text.color": "#E6E6E6",
        "axes.labelcolor": "#E6E6E6",
        "xtick.color": "#E6E6E6",
        "ytick.color": "#E6E6E6",
        "axes.edgecolor": "#666666",
        "grid.color": "#333333",
        "legend.facecolor": "#111111",
        "legend.edgecolor": "#444444",
    }
)

# Maps from vertex id -> (x, y)
poses = {}

show_pose_landmark_edges = True
show_odmetry_edges = False

# Lists of (from_id, to_id) for different edge types
odom_edges = []
landmark_edges = []

# Map from landmark id -> (x, y) (used for drawing observation edges)
landmarks = {}

blue_landmarks = {}
yellow_landmarks = {}
orange_landmarks = {}
big_orange_landmarks = {}

with open("/home/andre-lopes/Desktop/ros2_ws/optimized_graph.g2o", "r") as f:
    for line in f:
        parts = line.split()
        if not parts:
            continue

        tag = parts[0]

        # Vertices
        if tag == "VERTEX_SE2":
            vid = int(parts[1])
            x = float(parts[2])
            y = float(parts[3])
            poses[vid] = (x, y)
        elif tag in ("VERTEX_XY", "VERTEX_POINT_XY", "VERTEXLANDMARK2D"):
            vid = int(parts[1])
            x = float(parts[2])
            y = float(parts[3])
            color = int(parts[4])
            landmarks[vid] = (x, y)
            if color==1 :
                yellow_landmarks[vid] = (x, y, color)
            elif color==2 :
                blue_landmarks[vid] = (x,y, color)
            elif color==3 :
                orange_landmarks[vid] = (x,y, color)
            elif color==4 :
                big_orange_landmarks[vid] = (x,y, color)

        # Edges
        elif tag == "EDGE_SE2":
            # odometry between two poses
            from_id = int(parts[1])
            to_id = int(parts[2])
            odom_edges.append((from_id, to_id))
        elif tag in ("EDGE_SE2_XY", "EDGE_SE2_POINT_XY"):
            # observation from pose to landmark
            from_id = int(parts[1])
            to_id = int(parts[2])
            landmark_edges.append((from_id, to_id))

# Prepare arrays for plotting
poses_x = [p[0] for _, p in sorted(poses.items())]
poses_y = [p[1] for _, p in sorted(poses.items())]
blue_landmarks_x = [p[0] for _, p in sorted(blue_landmarks.items())]
blue_landmarks_y = [p[1] for _, p in sorted(blue_landmarks.items())]

yellow_landmarks_x = [p[0] for _, p in sorted(yellow_landmarks.items())]
yellow_landmarks_y = [p[1] for _, p in sorted(yellow_landmarks.items())]

orange_landmarks_x = [p[0] for _, p in sorted(orange_landmarks.items())]
orange_landmarks_y = [p[1] for _, p in sorted(orange_landmarks.items())]

big_orange_landmarks_x = [p[0] for _, p in sorted(big_orange_landmarks.items())]
big_orange_landmarks_y = [p[1] for _, p in sorted(big_orange_landmarks.items())]


plt.scatter(blue_landmarks_x, blue_landmarks_y, c="#107AEB", s=20, label="Blue Landmarks")
plt.scatter(yellow_landmarks_x, yellow_landmarks_y, c="#E4F900", s=20, label="Yellow Landmarks")
plt.scatter(orange_landmarks_x, orange_landmarks_y, c="#E58703", s=20, label="Orange Landmarks")
plt.scatter(big_orange_landmarks_x, big_orange_landmarks_y, c="red", s=20, label="Big Orange Landmarks")

# Plot pose trajectory
if poses_x:
    plt.scatter(poses_x, poses_y, c="#FFFFFF", marker="o", s=3, label="Trajectory")

# Plot odometry edges between poses
if show_odmetry_edges:
    for from_id, to_id in odom_edges:
        if from_id in poses and to_id in poses:
            x1, y1 = poses[from_id]
            x2, y2 = poses[to_id]
            plt.plot([x1, x2], [y1, y2], c="gray", linewidth=0.5, alpha=0.7)

# Plot landmark observation edges
if show_pose_landmark_edges:
    for from_id, to_id in landmark_edges:
        if from_id in poses and to_id in landmarks:
            x1, y1 = poses[from_id]
            x2, y2 = landmarks[to_id]
            plt.plot([x1, x2], [y1, y2], c="#53F4B6FF", linewidth=0.4, alpha=0.5)

plt.legend(loc="upper right")
plt.axis("equal")
plt.show()
