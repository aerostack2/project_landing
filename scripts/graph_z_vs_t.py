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
    pose_z = pose_z[3400:8300]
    pose_z_sim = pose_z_sim[600:3872]

    # Crear la figura y el eje 3D
    t = np.linspace(0, 1, 4900)
    t_sim = np.linspace(0,1,3272)

    plt.figure(figsize=(8, 5))
    plt.plot(t, pose_z, label='Z real', color='b', linestyle='-')
    plt.plot(t_sim, pose_z_sim, label='Z sim', color='r', linestyle='-')

    plt.xlabel('Tiempo (s)')
    plt.ylabel('Z (m)')
    plt.title('Gráfica de altura en Z frente al tiempo')
    plt.legend()
    plt.grid(True)

    # Mostrar el gráfico
    plt.show()
