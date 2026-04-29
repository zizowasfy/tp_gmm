import json
import os

nb_path = "/home/zizo/the_folder/ws_moveit/src/tp_gmm/scripts/demons_to_samples_ur10_demons.ipynb"

wrapper_code = """from rosbags.highlevel import AnyReader

class RosbagWrapper:
    def __init__(self, path):
        from pathlib import Path
        self.path = path
        self.reader = AnyReader([Path(self.path)])
        self.reader.open()
        
    def read_messages(self, topics):
        connections = [x for x in self.reader.connections if x.topic in topics]
        for connection, timestamp, rawdata in self.reader.messages(connections=connections):
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield connection.topic, msg, timestamp
            
    def close(self):
        self.reader.close()
"""

with open(nb_path, 'r') as f:
    nb = json.load(f)

for cell in nb['cells']:
    if cell['cell_type'] == 'code':
        source = cell['source']
        for i, line in enumerate(source):
            if "import rosbag2_py\n" in line:
                # Replace the entire cell contents up to "import numpy as np" with the new wrapper
                for j, sub_line in enumerate(source):
                    if "import numpy as np" in sub_line:
                        break
                source[:j] = [wrapper_code]
                break

with open(nb_path, 'w') as f:
    json.dump(nb, f, indent=1)

print("Restored notebook to use the rosbags package successfully")
