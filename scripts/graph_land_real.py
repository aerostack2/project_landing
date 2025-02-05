import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from scipy.interpolate import interp1d
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

def get_rosbag_options(bag_path, storage_id='sqlite3', serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=storage_id
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format
    )
    return storage_options, converter_options

def resample_trajectory(traj, new_length):
    old_length = len(traj)
    x_old = np.linspace(0, 1, old_length)
    x_new = np.linspace(0, 1, new_length)
    interpolator = interp1d(x_old, traj, axis=0, kind='linear')
    return interpolator(x_new)

def plot_2d(x1, y1, x2, y2, title="Gráfica 2D", xlabel="Eje X", ylabel="Eje Y", legend_label1=None, legend_label2=None):
    """
    Función para representar datos en 2D con un formato uniforme.
    """
    fig, ax = plt.subplots(figsize=(8, 6))  # Tamaño uniforme
    ax.plot(x1, y1, color='blue', label=legend_label1)
    ax.plot(x2, y2, color='red', label=legend_label2)
    ax.set_title(title, fontsize=14)
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.grid(True)
    
    ax.legend(fontsize=14)
    
    plt.show()

def plot_3d(x1, y1, z1, x2, y2, z2, x3, y3, z3, title="Gráfica 3D", xlabel="Eje X", ylabel="Eje Y", zlabel="Eje Z", legend_label1=None, legend_label2=None, legend_label3 = None):
    """
    Función para representar datos en 3D con un formato uniforme.
    """
    fig = plt.figure(figsize=(8, 6))  # Tamaño uniforme
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x1, y1, z1, color='blue', label=legend_label1)
    ax.plot(x2, y2, z2, color='orange', label=legend_label2)
    ax.plot(x3, y3, z3, color='red', label=legend_label3)
    ax.set_title(title, fontsize=14)
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_zlabel(zlabel, fontsize=12)
    
    # ax.legend(fontsize=12)
    
    plt.show()

if __name__ == "__main__":

    # Open rosbag to read data
    RESOURCES_PATH = Path('/home/keikei/project_landing/real_model/rosbag/rosbags/')

    rosbag = 'landing/rosbag2_2025_01_29-19_12_02/rosbag2_2025_01_29-19_12_02_0.db3'

    bag_path = str(RESOURCES_PATH / rosbag)

    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for the topic of position during trajectory
    storage_filter = rosbag2_py.StorageFilter(topics=['/drone0/self_localization/pose', '/drone0/motion_reference/trajectory', '/drone0/debug/traj_generated'])
    reader.set_filter(storage_filter)

    # Store pose data for the plot
    stamps = []
    pose_x = []
    pose_y = []
    pose_z = []

    pose_x_coche = []
    pose_y_coche = []
    pose_z_coche = []    

    trajectory_x = []
    trajectory_y = []
    trajectory_z = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == '/drone0/self_localization/pose':
            pose_x.append(msg.pose.position.x)
            pose_y.append(msg.pose.position.y)
            pose_z.append(msg.pose.position.z)
        elif topic == '/drone0/motion_reference/trajectory':
            trajectory_x.append(-(msg.setpoints[0].position.y ) - 0.62)
            trajectory_y.append((msg.setpoints[0].position.x) + 3.32)
            trajectory_z.append((msg.setpoints[0].position.z) + 0.11)
        elif topic == '/drone0/debug/traj_generated':
                pose_x_coche.append(-(msg.poses[-1].pose.position.y) - 0.62 + 0.05)
                pose_y_coche.append((msg.poses[-1].pose.position.x) + 3.32 + 0.05)
                pose_z_coche.append(0.5 + 0.11)

    # trim trajectory
    # pose_x = pose_x[3400:7200]
    # pose_x_coche = pose_x_coche[600:3872]
    # pose_y = pose_y[3400:7200]
    # pose_y_coche = pose_y_coche[600:3872]
    # pose_z = pose_z[3400:7200]
    # pose_z_coche = pose_z_coche[600:3872]

    pose_x_coche = pose_x_coche[1:]
    pose_y_coche = pose_y_coche[1:]
    pose_z_coche = pose_z_coche[1:]


    title = "Trayectorias 3D en el aterrizaje real"
    xlabel = "Eje X (m)"
    ylabel = "Eje Y (m)"
    zlabel = "Eje Z (m)"

    plot_3d(pose_x, pose_y, pose_z, pose_x_coche, pose_y_coche, pose_z_coche, trajectory_x, trajectory_y, trajectory_z, title, xlabel, ylabel, zlabel, "", "", "")

    new_length = max(len(pose_x), len(pose_x_coche))
    pose_x_coche = resample_trajectory(pose_x_coche, new_length)
    pose_y_coche = resample_trajectory(pose_y_coche, new_length)
    pose_z_coche = resample_trajectory(pose_z_coche, new_length)

    t = np.linspace(0, 1, len(pose_x))

    dist_xy = np.sqrt((np.array(pose_x) - np.array(pose_x_coche)) ** 2, (np.array(pose_y) - np.array(pose_y_coche)) ** 2)
    dist_z = np.array(pose_z) - np.array(pose_z_coche)

    title_dist = "Distancias entre el dron y el coche"
    dist_label = "Distancia (m)"
    tlabel = "Tiempo"
    legend_dist1 = "Distancia en el plano XY"
    legend_dist2 = "Distancia de altura"

    plot_2d(t, dist_xy, t, dist_z, title_dist, tlabel, dist_label, legend_dist1, legend_dist2)