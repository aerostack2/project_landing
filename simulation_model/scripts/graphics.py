import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
import scipy
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

if __name__ == "__main__":

    # Open rosbag to read data
    RESOURCES_PATH = Path('/home/keikei/project_landing/real_model/rosbag/rosbags/')

    rosbag = 'flight_rosbags/rosbag2_2025_01_16-17_56_59/rosbag2_2025_01_16-17_56_59_0.db3'

    bag_path = str(RESOURCES_PATH / rosbag)

    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for the topic of position during trajectory
    storage_filter = rosbag2_py.StorageFilter(topics=['/drone0/self_localization/pose', '/drone0/motion_reference/trajectory'])
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
        elif topic == '/drone0/motion_reference/trajectory':
            trajectory_x.append(msg.setpoints[0].position.y)
            trajectory_y.append(msg.setpoints[0].position.x)
            trajectory_z.append(msg.setpoints[0].position.z)


    # Crear la figura y el eje 3D
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Graficar la trayectoria
    ax.plot(pose_x, pose_y, pose_z, label='Trayectoria 3D', color='b', linewidth=2)
    ax.plot(trajectory_x, trajectory_y, trajectory_z, color='r', linewidth=2)

    # Configurar etiquetas de los ejes
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')
    ax.set_zlabel('Eje Z')

    # Agregar título y leyenda
    ax.set_title('Gráfica de Trayectoria en 3D')
    ax.legend()

    # Mostrar el gráfico
    plt.show()
