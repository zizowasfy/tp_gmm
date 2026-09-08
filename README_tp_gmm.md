# TP-GMM (Task Parameterized Gaussian Mixture Model) Pipeline

This package provides a Task-Parameterized Gaussian Mixture Model (TP-GMM) implementation for robot motion planning and learning from demonstration in ROS 2. It takes human demonstrations, learns a generalized statistical model of the task conditioned on frames (like start and goal poses), and reproduces a motion path adapted to new frame configurations.

## Codebase Structure

The `tp_gmm` package is structured into several key directories:

### 1. `include/`
Contains the core mathematical and algorithmic implementations of the TP-GMM pipeline in Python.
- **`TPGMM_GMR.py`**: The main interface class for fitting the model (`fit`) and reproducing paths (`reproduce`).
- **`EM_tensorGMM.py`**: Implementation of the Expectation-Maximization algorithm adapted for Tensor/Task-Parameterized GMMs.
- **`init_proposedPGMM_timeBased.py`**: Initialization logic for the model before running the EM algorithm.
- **`reproduction_DSGMR.py`**: Gaussian Mixture Regression (GMR) used to generate the reproduced trajectory based on the learned GMM and the new task frames.
- **Data structure classes** (`sClass.py`, `pClass.py`, `rClass.py`, `modelClass.py`): Encapsulate points, frames, and model parameters for clean object-oriented handling.

### 2. `scripts/`
Contains the ROS 2 nodes, service servers/clients, and scripts to process the raw demonstrations.
- **`tp_gmm_node.py`**: The main ROS 2 node. It provides two services: `StartTPGMM_service` (trains the model) and `ReproduceTPGMM_service` (generates constraints/paths for MoveIt based on new frames).
- **`demons_to_samples.py`**: A module that extracts raw trajectory data from ROS 2 bags, synchronizes the points using Dynamic Time Warping (DTW), calculates transformations relative to the start/goal frames, and passes the matrices into memory for training.
- **`collect_trajectories_gen3_ros2.py`**: Script for collecting trajectory demonstrations by executing Cartesian planning in MoveIt and recording the path/frames into ROS bags using the `rosbags` package.

### 3. `tasks/` & `data/`
- **`tasks/`**: Stores information about the specific tasks (e.g., `pick`, `place`). This is where the trained `TPGMM_model.pkl` and `demons_info.pkl` are saved for later reproduction without retraining.
- **`data/`**: (Deprecated in optimized workflow) Historically used to store the intermediate `_A.txt`, `_b.txt`, and `_Data.txt` files which are now processed directly in memory for speed.

### 4. `launch/`
- Contains launch files to spin up the `tp_gmm_node`, visualization tools, and other dependencies.

---

## Instructions: How to Run the Pipeline

### Step 1: Collect Demonstrations
First, you need to collect demonstrations for your task (e.g., `pick`). This creates ROS 2 bags recording the `start_pose`, `goal_pose`, and the actual trajectory.

```bash
ros2 run tp_gmm collect_trajectories_gen3_ros2.py
```
*(You may need to modify the task string in the main block of the script depending on what you're recording).*

### Step 2: Launch the TP-GMM Node
Launch the main node that provides the training and reproduction services.

```bash
ros2 launch tp_gmm lfd_launch.py
```
Or run the node directly:
```bash
ros2 run tp_gmm tp_gmm_node.py
```

### Step 3: Train the Model (Start TPGMM)
To process the collected demonstrations and learn the GMM parameters, call the `StartTPGMM` service. This will execute `demons_to_samples.py` in the background and fit the EM model.

```bash
ros2 service call /StartTPGMM_service tp_gmm/srv/StartTPGMM "{task_name: 'pick'}"
```
*Note: This will save the trained model into the `tasks/pick/TPGMM_model.pkl` file.*

### Step 4: Reproduce the Trajectory (Inference)
Once the model is trained, you can pass new `start_pose` and `goal_pose` frames to the `ReproduceTPGMM_service`. The service will output a generated trajectory and publish the GMM bounding volumes to guide MoveIt constraint-based planning.

*Typically, this service is called programmatically by a motion planning script (e.g., `collect_trajectories_gen3_ros2.py` calls it to test constraints), but it can be called manually if you construct the geometry poses properly.*

---
## Recent Performance Refactoring
- Removed `papermill` overhead by converting Jupyter notebook (`demons_to_samples_ur10_demons.ipynb`) into a pure Python module.
- Replaced slow file I/O operations (`np.loadtxt` on thousands of `.txt` files) with in-memory parameter passing between data extraction and `tpGMM` training.
- Reduced inference time by caching `TPGMM_model` as a class attribute in `tp_gmm_node.py`.

## Cartesian and joint-projected GMM sampling

See [SAMPLING.md](SAMPLING.md) for the fresh `sampling-approaches-cod` implementation, RViz laboratory, live mode switching, paired benchmark, mathematics and validation commands.
