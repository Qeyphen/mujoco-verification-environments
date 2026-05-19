#!/usr/bin/env python3
"""
Model Inspector for MuJoCo MJCF Files
Usage: python model_inspector.py <path_to_xml>
"""

import sys
import mujoco
import numpy as np
from pathlib import Path


def inspect_model(xml_path: str) -> None:
    """
    Comprehensive model diagnostic tool.
    Reports compilation status, physical properties, and structural details.
    """
    xml_file = Path(xml_path)
    
    # Validation: File existence check
    if not xml_file.exists():
        print(f"❌ ERROR: File not found: {xml_path}")
        sys.exit(1)
    
    print(f"📋 Inspecting: {xml_file.name}")
    print("=" * 60)
    
    try:
        # Load and compile model
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        
        print("✅ Compilation: SUCCESS\n")
        
        print("🔗 KINEMATIC STRUCTURE")
        print(f"   Bodies:     {model.nbody}")
        print(f"   Joints:     {model.njnt}")
        print(f"   DOFs (nv):  {model.nv}")
        print(f"   Geoms:      {model.ngeom}")
        print()
        
        print("⚙️  ACTUATION SYSTEM")
        print(f"   Actuators:  {model.nu}")
        if model.nu > 0:
            print("   Actuator Details:")
            for i in range(model.nu):
                act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if act_name is None:
                  act_name = f"actuator_{i}" 
                  
                # Get control range if limited
                if model.actuator_ctrllimited[i]:
                    ctrl_min = model.actuator_ctrlrange[i, 0]
                    ctrl_max = model.actuator_ctrlrange[i, 1]
                    print(f"      [{i}] {act_name:20s} | Range: [{ctrl_min:6.1f}, {ctrl_max:6.1f}]")
                else:
                    print(f"      [{i}] {act_name:20s} | Range: UNLIMITED ⚠️")
        print()
        
        print("📡 SENSOR SYSTEM")
        print(f"   Sensors:    {model.nsensor}")
        if model.nsensor > 0:
            print("   Sensor Details:")
            for i in range(model.nsensor):
                sensor_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
                sensor_type = model.sensor_type[i]
                sensor_dim = model.sensor_dim[i]
                print(f"      [{i}] {sensor_name:20s} | Type: {sensor_type} | Dim: {sensor_dim}")
        print()
        
        print("⚖️  PHYSICAL PROPERTIES")
        total_mass = np.sum(model.body_mass)
        print(f"   Total Mass: {total_mass:.3f} kg")
        
        # Find heaviest body
        if model.nbody > 1:
            body_masses = model.body_mass[1:]  # Skip worldbody (index 0)
            heaviest_idx = np.argmax(body_masses) + 1  # Add 1 back
            heaviest_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, heaviest_idx)
            heaviest_mass = model.body_mass[heaviest_idx]
            print(f"   Heaviest Body: {heaviest_name} ({heaviest_mass:.3f} kg)")
        print()
        
        print("⏱️  SIMULATION SETTINGS")
        print(f"   Timestep:   {model.opt.timestep:.4f} s")
        print(f"   Gravity:    [{model.opt.gravity[0]:.2f}, {model.opt.gravity[1]:.2f}, {model.opt.gravity[2]:.2f}] m/s²")
        print()
        
        print("🤝 CONTACT CONFIGURATION")
        print(f"   Contact Pairs: {model.npair}")
        print(f"   Excludes:      {model.nexclude}")
        print()
        
        print("⚠️  DIAGNOSTICS")
        warnings = []
        
        unlimited_actuators = np.sum(~model.actuator_ctrllimited)
        if unlimited_actuators > 0:
            warnings.append(f"   ⚠️  {unlimited_actuators} actuator(s) have UNLIMITED control range (security risk)")
        
        zero_mass_bodies = np.sum(model.body_mass == 0)
        if zero_mass_bodies > 1:  # worldbody is always zero
            warnings.append(f"   ⚠️  {zero_mass_bodies - 1} body/bodies have ZERO mass")
        
        if model.opt.timestep > 0.01:
            warnings.append(f"   ⚠️  Large timestep ({model.opt.timestep}s) may cause instability")
        
        if model.nsensor == 0 and model.nu > 0:
            warnings.append("   ℹ️  No sensors defined (useful for observation spaces)")
        
        if warnings:
            for warning in warnings:
                print(warning)
        else:
            print("   ✅ No issues detected")
        
        print("=" * 60)
        
    except Exception as error:
        print(f"❌ Compilation FAILED")
        print(f"Error: {error}")
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python model_inspector.py <path_to_xml>")
        print("Example: python model_inspector.py planar_pole.xml")
        sys.exit(1)
    
    inspect_model(sys.argv[1])