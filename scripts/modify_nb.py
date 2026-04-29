import json
import os

nb_path = "/home/zizo/the_folder/ws_moveit/src/tp_gmm/scripts/demons_to_samples_ur10_demons.ipynb"

wrapper_code = """import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class RosbagWrapper:
    def __init__(self, path):
        self.path = path
        
    def read_messages(self, topics):
        storage_options = rosbag2_py.StorageOptions(uri=self.path)
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        storage_filter = rosbag2_py.StorageFilter(topics=topics)
        reader.set_filter(storage_filter)
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: get_message(t.type) for t in topic_types}
        
        while reader.has_next():
            topic, data, t = reader.read_next()
            msg = deserialize_message(data, type_map[topic])
            yield topic, msg, t
            
    def close(self):
        pass
"""

with open(nb_path, 'r') as f:
    nb = json.load(f)

for cell in nb['cells']:
    if cell['cell_type'] == 'code':
        source = cell['source']
        for i, line in enumerate(source):
            if "import rosbag\n" in line:
                source[i] = line.replace("import rosbag\n", wrapper_code)
            if "demon_bag = rosbag.Bag" in line:
                source[i] = line.replace("rosbag.Bag", "RosbagWrapper").replace(", 'r'", "")

with open(nb_path, 'w') as f:
    json.dump(nb, f, indent=1)

print("Modified notebook to use rosbag2_py successfully")
