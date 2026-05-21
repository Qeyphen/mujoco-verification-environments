import mujoco
import numpy as np

class TrajectoryLogger:
    """
    A reusable logger for safely recording MuJoCo simulation trajectories.
    
    Ensures all data is properly copied to avoid memory aliasing bugs.
    Validates for NaN values during recording.
    """
    
    def __init__(self, model):
        self.model = model
        self.trajectory = []
    
    def record(self, data):
        qpos = data.qpos.copy()
        qvel = data.qvel.copy()
        ctrl = data.ctrl.copy()
        time = float(data.time)
        
        # Validate for NaN values
        if np.any(np.isnan(qpos)):
            raise ValueError(f"NaN detected in qpos at time {time}")
        if np.any(np.isnan(qvel)):
            raise ValueError(f"NaN detected in qvel at time {time}")
        if np.any(np.isnan(ctrl)):
            raise ValueError(f"NaN detected in ctrl at time {time}")
        if np.isnan(time):
            raise ValueError("NaN detected in time")
        
        self.trajectory.append({
            "qpos": qpos,
            "qvel": qvel,
            "ctrl": ctrl,
            "time": time
        })
    
    def get_trajectory(self):
        return self.trajectory
    
    def clear(self):
        self.trajectory = []
    
    def __len__(self):
        return len(self.trajectory)
    
    def get_qpos_array(self):
        if not self.trajectory:
            return np.array([])
        return np.array([step['qpos'] for step in self.trajectory])
    
    def get_qvel_array(self):
        if not self.trajectory:
            return np.array([])
        return np.array([step['qvel'] for step in self.trajectory])
    
    def get_times(self):
        return np.array([step['time'] for step in self.trajectory])


def test_trajectory_logger():
    xml = """
    <mujoco>
        <worldbody>
            <body>
                <joint name="hinge" type="hinge"/>
                <geom type="capsule" size="0.01" fromto="0 0 0 0 0 -0.3"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="hinge" ctrllimited="true" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    print("Test 1: Basic recording...")
    logger = TrajectoryLogger(model)
    
    for i in range(10):
        data.ctrl[0] = i * 0.1
        mujoco.mj_step(model, data)
        logger.record(data)
    
    assert len(logger) == 10, "Should have 10 timesteps"
    print("✓ Recorded 10 timesteps")
    
    print("\nTest 2: Memory aliasing check...")
    logger.clear()
    
    # Record same position twice
    data.qpos[0] = 1.0
    logger.record(data)
    
    data.qpos[0] = 2.0  # Change data
    logger.record(data)
    
    traj = logger.get_trajectory()
    first_qpos = traj[0]['qpos'][0]
    second_qpos = traj[1]['qpos'][0]
    
    assert first_qpos == 1.0, f"First should be 1.0, got {first_qpos}"
    assert second_qpos == 2.0, f"Second should be 2.0, got {second_qpos}"
    assert first_qpos != second_qpos, "Memory aliasing detected!"
    print(f"✓ No memory aliasing: first={first_qpos}, second={second_qpos}")
    
    print("\nTest 3: Convenience methods...")
    logger.clear()
    
    for i in range(5):
        data.qpos[0] = i
        data.qvel[0] = i * 2
        data.time = i * 0.01
        logger.record(data)
    
    qpos_array = logger.get_qpos_array()
    qvel_array = logger.get_qvel_array()
    times = logger.get_times()
    
    assert qpos_array.shape == (5, model.nq), f"Wrong qpos shape: {qpos_array.shape}"
    assert qvel_array.shape == (5, model.nv), f"Wrong qvel shape: {qvel_array.shape}"
    assert times.shape == (5,), f"Wrong times shape: {times.shape}"
    print(f"✓ Array shapes correct: qpos={qpos_array.shape}, qvel={qvel_array.shape}, times={times.shape}")
    
    print("\nTest 4: NaN validation...")
    logger.clear()
    data.qpos[0] = np.nan
    
    try:
        logger.record(data)
        assert False, "Should have raised ValueError for NaN"
    except ValueError as e:
        print(f"✓ NaN detected correctly: {e}")
    
    print("\nTest 5: Clear functionality...")
    data.qpos[0] = 0.5  # Reset to valid value
    logger.record(data)
    assert len(logger) == 1, "Should have 1 entry"
    
    logger.clear()
    assert len(logger) == 0, "Should be empty after clear"
    print("✓ Clear works correctly")
    
    print("\nTest 6: Reusability across episodes...")
    for episode in range(3):
        logger.clear()
        for step in range(5):
            data.qpos[0] = episode + step * 0.1
            logger.record(data)
        assert len(logger) == 5, f"Episode {episode} should have 5 steps"
    print("✓ Logger reusable across episodes")
    
    print("\n" + "="*50)
    print("ALL TESTS PASSED! ✓")
    print("="*50)


if __name__ == "__main__":
    test_trajectory_logger()