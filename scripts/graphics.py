import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from scipy.interpolate import interp1d
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

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
    ax.plot(x1, y1, color='blue',label=legend_label1)
    ax.plot(x2, y2, color='red', label=legend_label2)
    ax.set_title(title, fontsize=14)
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.grid(True)
    
    ax.legend(fontsize=12)
    
    plt.show()

def plot_3d(x1, y1, z1, x2, y2, z2, title="Gráfica 3D", xlabel="Eje X", ylabel="Eje Y", zlabel="Eje Z", legend_label1=None, legend_label2=None):
    """
    Función para representar datos en 3D con un formato uniforme.
    """
    fig = plt.figure(figsize=(8, 6))  # Tamaño uniforme
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x1, y1, z1, color='blue', label=legend_label1)
    ax.plot(x2, y2, z2, color='red', label=legend_label2)
    ax.set_title(title, fontsize=14)
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_zlabel(zlabel, fontsize=12)
    
    ax.legend(fontsize=12)
    
    plt.show()

def mean_squared_error(x, y):
    """
    Calcula el Error Cuadrático Medio entre los puntos definidos por las listas x e y.
    """
    if len(x) != len(y):
        raise ValueError("Las listas x e y deben tener la misma longitud")
    
    mse = np.mean((np.array(x) - np.array(y)) ** 2)
    return mse

def mean_squared_error_trajectories(x1, y1, x2, y2):
    """
    Calcula el Error Cuadrático Medio entre dos trayectorias definidas por (x1, y1) y (x2, y2).
    """
    if len(x1) != len(y1) or len(x2) != len(y2):
        raise ValueError("Las listas de coordenadas deben tener la misma longitud")
    if len(x1) != len(x2):
        raise ValueError("Las dos trayectorias deben tener la misma cantidad de puntos")
    
    mse = np.mean((np.array(x1) - np.array(x2)) ** 2 + (np.array(y1) - np.array(y2)) ** 2)
    return mse

if __name__ == "__main__":

    # Open rosbag to read data
    RESOURCES_PATH = Path('/home/keikei/project_landing/real_model/rosbag/rosbags/')

    rosbag_real = 'flight_rosbags/rosbag2_2025_01_16-17_56_59/rosbag2_2025_01_16-17_56_59_0.db3'

    bag_path = str(RESOURCES_PATH / rosbag_real)
    bag_path_sim = '/home/keikei/project_landing/simulation_model/rosbag/vuelo_sim_2/vuelo_sim_2_0.db3'

    storage_options, converter_options = get_rosbag_options(bag_path)
    storage_options_sim, converter_options_sim = get_rosbag_options(bag_path_sim)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    reader_sim = rosbag2_py.SequentialReader()
    reader_sim.open(storage_options_sim, converter_options_sim)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for the topic of position during trajectory
    # storage_filter = rosbag2_py.StorageFilter(topics=['/drone0/self_localization/pose', '/drone0/motion_reference/trajectory'])
    storage_filter = rosbag2_py.StorageFilter(topics=['/drone0/self_localization/pose'])
    reader.set_filter(storage_filter)

    # Store pose data for the plot
    stamps = []
    pose_x = []
    pose_y = []
    pose_z = []

    trajectory_x = []
    trajectory_y = []
    trajectory_z = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == '/drone0/self_localization/pose':
            stamps.append(float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-09)
            pose_x.append(msg.pose.position.x)
            pose_y.append(msg.pose.position.y)
            pose_z.append(msg.pose.position.z)
        # elif topic == '/drone0/motion_reference/trajectory':
        #     trajectory_x.append(msg.setpoints[0].position.y)
        #     trajectory_y.append(msg.setpoints[0].position.x)
        #     trajectory_z.append(msg.setpoints[0].position.z)


    # Process simulation rosbag
    topic_types_sim = reader_sim.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map_sim = {topic_types_sim[i].name: topic_types_sim[i].type for i in range(len(topic_types_sim))}

    # Set filter for the topic of position during trajectory
    storage_filter = rosbag2_py.StorageFilter(topics=['/drone0/self_localization/pose'])
    reader_sim.set_filter(storage_filter)

    # Store pose data for the plot
    pose_x_sim = []
    pose_y_sim = []
    pose_z_sim = []

    # trajectory_x_sim = []
    # trajectory_y_sim = []
    # trajectory_z_sim = []

    while reader_sim.has_next():
        (topic, data, t) = reader_sim.read_next()
        msg_type = get_message(type_map_sim[topic])
        msg = deserialize_message(data, msg_type)

        # if topic == '/drone0/self_localization/pose':
        pose_x_sim.append(msg.pose.position.x)
        pose_y_sim.append(msg.pose.position.y)
        pose_z_sim.append(msg.pose.position.z)
        # elif topic == '/drone0/motion_reference/trajectory':
        #     trajectory_x.append(msg.setpoints[0].position.y)
        #     trajectory_y.append(msg.setpoints[0].position.x)
        #     trajectory_z.append(msg.setpoints[0].position.z)

    # trim trajectory
    pose_z_trim = pose_z[3400:8300]
    pose_z_sim_trim = pose_z_sim[600:3872]

    # trim trajectory
    pose_x = pose_x[3400:7200]
    pose_x_sim = pose_x_sim[600:3872]
    pose_y = pose_y[3400:7200]
    pose_y_sim = pose_y_sim[600:3872]
    pose_z = pose_z[3400:7200]
    pose_z_sim = pose_z_sim[600:3872]



    new_length = max(len(pose_x), len(pose_x_sim))
    pose_x_fixed = resample_trajectory(pose_x, new_length)
    pose_y_fixed = resample_trajectory(pose_y, new_length)
    pose_z_fixed = resample_trajectory(pose_z, new_length)
    pose_x_sim_fixed = resample_trajectory(pose_x_sim, new_length)
    pose_y_sim_fixed = resample_trajectory(pose_y_sim, new_length)
    pose_z_sim_fixed = resample_trajectory(pose_z_sim, new_length)


    # # Crear la figura y el eje 3D
    # fig = plt.figure(figsize=(8, 6))
    # ax = fig.add_subplot(111, projection='3d')

    # # Graficar la trayectoria
    # ax.plot(pose_x, pose_y, pose_z, label='Trayectoria real', color='b', linewidth=2)
    # ax.plot(pose_x_sim, pose_y_sim, pose_z_sim, label='Trayectoria simulada', color='r', linewidth=2)

    # # Configurar etiquetas de los ejes
    # ax.set_xlabel('Eje X')
    # ax.set_ylabel('Eje Y')
    # ax.set_zlabel('Eje Z')

    # # Agregar título y leyenda
    # ax.set_title('Gráfica de Trayectoria en 3D')
    # ax.legend()

    # # Mostrar el gráfico
    # plt.show()

    title = "Trayectorias 3D"
    xlabel = "Eje X (m)"
    ylabel = "Eje Y (m)"
    zlabel = "Eje Z (m)"
    label1 = "Trayectoria del dron real"
    label2 = "Trayectoria del dron en simulación"
    tlabel = "Tiempo"

    titleXY = "Trayectorias en plano XY"
    titleZ = "Trayectorias en altura"

    # Crear la figura y el eje 3D
    t = np.linspace(0, 1, 4900)
    t_sim = np.linspace(0,1,3272)

    plot_3d(pose_x_fixed, pose_y_fixed, pose_z_fixed, pose_x_sim_fixed, pose_y_sim_fixed, pose_z_sim_fixed, title, xlabel, ylabel, zlabel, label1, label2)
    plot_2d(pose_x_fixed, pose_y_fixed, pose_x_sim_fixed, pose_y_sim_fixed, titleXY, xlabel, ylabel, label1, label2)
    plot_2d(t, pose_z_trim, t_sim, pose_z_sim_trim, titleZ, tlabel, zlabel,label1, label2)

    print(mean_squared_error_trajectories(pose_x_fixed, pose_y_fixed, pose_x_sim_fixed, pose_y_sim_fixed)/100)
    print(mean_squared_error(pose_z_fixed, pose_z_sim_fixed))
