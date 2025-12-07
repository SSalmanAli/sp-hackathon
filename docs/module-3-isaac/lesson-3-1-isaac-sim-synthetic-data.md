---
title: "Lesson 3.1: NVIDIA Isaac Sim and Synthetic Data"
sidebar_position: 1
description: "Using Isaac Sim for photorealistic simulation and synthetic data generation"
---

# Lesson 3.1: NVIDIA Isaac Sim and Synthetic Data

## Context
This lesson introduces NVIDIA Isaac Sim, a powerful robotics simulation platform built on Omniverse. Isaac Sim enables the creation of photorealistic environments and synthetic data for training AI models, which is essential for developing robust perception systems in Physical AI.

## What You'll Build
In this lesson, you'll create an Isaac Sim environment that generates synthetic training data for perception models. You'll learn to configure realistic lighting, materials, and sensor models to create diverse datasets for AI training.

## Required Tools
- NVIDIA Isaac Sim
- Omniverse
- Isaac ROS
- NVIDIA GPU with CUDA support
- ROS 2 (Humble Hawksbill)

## Important Concepts

### Synthetic Data Generation
Creating training data using simulated environments, which can provide diverse scenarios and ground truth annotations.

### Physically-Based Rendering (PBR)
Rendering techniques that simulate real-world lighting and material properties for photorealistic results.

### Domain Randomization
Randomly varying environmental parameters (lighting, textures, objects) to make AI models more robust.

### Sensor Simulation
Accurate modeling of real-world sensors including noise, distortion, and other imperfections.

## Implementation Steps

### Step 1: Set Up Isaac Sim Environment
1. Install Isaac Sim and Omniverse
2. Configure the simulation environment with basic lighting
3. Test the basic setup with a simple robot model

### Step 2: Create Photorealistic Scene
1. Design a scene with diverse objects and materials
2. Configure physically-based lighting
3. Set up realistic textures and surfaces

### Step 3: Configure Sensor Simulation
1. Add camera sensors with realistic parameters
2. Configure LIDAR and other sensors
3. Set up sensor noise models and calibration

### Step 4: Generate Synthetic Data
1. Create scripts to randomize scene parameters
2. Capture sensor data and ground truth annotations
3. Export data in formats suitable for AI training

## Code Example: Isaac Sim Scene Setup

```python
import omni
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.carb import carb_settings
from omni.isaac.sensor import Camera
from pxr import Gf, Sdf, UsdGeom, UsdShade
import numpy as np

# Initialize the Isaac Sim world
world = World(stage_units_in_meters=1.0)

# Add a simple robot to the scene
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Franka/franka_alt_fingers.usd",
    prim_path="/World/Robot"
)

# Create a ground plane
ground_plane = world.scene.add_ground_plane("/World/GroundPlane", size=10.0)

# Add random objects to the scene
import random
object_paths = [
    "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Blocks/block_01_20cm.usd",
    "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Kinova/kortex_allegro.usd",
    "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Towers/tower_01.usd"
]

for i in range(5):
    obj_path = random.choice(object_paths)
    obj_prim_path = f"/World/Object_{i}"
    add_reference_to_stage(usd_path=obj_path, prim_path=obj_prim_path)

    # Randomize position
    x = random.uniform(-3, 3)
    y = random.uniform(-3, 3)
    z = 0.1  # Slightly above ground
    omni.kit.commands.TransformMultiPrimsSRTCommand(
        prim_paths=[obj_prim_path],
        translation_offsets=[[x, y, z]],
        usd_context=omni.usd.get_context()
    ).do()

# Set up lighting
light_prim_path = "/World/Light"
omni.kit.commands.CreateLightCommand(
    light_type="DomeLight",
    prim_path=light_prim_path,
    name="dome_light"
).do()

# Configure the dome light
dome_light = get_prim_at_path(light_prim_path)
dome_light.GetAttribute("inputs:color").Set(Gf.Vec3f(0.2, 0.2, 0.2))

print("Isaac Sim scene configured with robot, objects, and lighting")
```

## Code Example: Camera Sensor Configuration

```python
import omni
from omni.isaac.sensor import Camera
from omni.isaac.core import World
import numpy as np
import cv2
from PIL import Image
import os

def setup_camera_sensor(robot_prim_path="/World/Robot", camera_name="camera", resolution=(640, 480)):
    """
    Set up a camera sensor in Isaac Sim
    """
    # Create camera sensor
    camera = Camera(
        prim_path=f"{robot_prim_path}/camera",
        name=camera_name,
        translation=np.array([0.2, 0.0, 0.1]),  # Position relative to robot
        orientation=np.array([0, 0, 0, 1]),  # Default orientation
        frequency=30,  # Hz
        resolution=resolution
    )

    # Add the camera to the world
    world = World.instance()
    world.scene.add(camera)

    # Configure camera parameters
    camera.add_motion_vectors_to_frame()
    camera.add_depth_to_frame()
    camera.add_instance_segmentation_to_frame()
    camera.add_bounding_box_2d_tight_to_frame()

    return camera

def capture_synthetic_data(camera, output_dir="synthetic_data", num_samples=100):
    """
    Capture synthetic data from the camera
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    world = World.instance()

    for i in range(num_samples):
        # Step the world to update sensor data
        world.step(render=True)

        # Get RGB image
        rgb_data = camera.get_rgb()
        if rgb_data is not None:
            # Convert to PIL Image and save
            rgb_image = Image.fromarray(rgb_data, mode="RGB")
            rgb_image.save(f"{output_dir}/rgb_{i:04d}.png")

        # Get depth data
        depth_data = camera.get_depth()
        if depth_data is not None:
            # Save depth as numpy array
            np.save(f"{output_dir}/depth_{i:04d}.npy", depth_data)

        # Get segmentation data
        seg_data = camera.get_segmentation()
        if seg_data is not None:
            # Convert to image and save
            seg_image = Image.fromarray(seg_data, mode="I")
            seg_image.save(f"{output_dir}/segmentation_{i:04d}.png")

        print(f"Captured sample {i+1}/{num_samples}")

    print(f"Synthetic data saved to {output_dir}")

# Example usage
if __name__ == "__main__":
    # Setup camera
    camera = setup_camera_sensor()

    # Capture synthetic data
    capture_synthetic_data(camera, output_dir="isaac_sim_data", num_samples=50)
```

## Code Example: Domain Randomization Script

```python
import random
import numpy as np
from pxr import Gf, UsdShade
import omni
from omni.isaac.core import World

class DomainRandomizer:
    def __init__(self, world, scene_objects):
        self.world = world
        self.scene_objects = scene_objects
        self.light_prim = None

    def randomize_lighting(self):
        """
        Randomize lighting conditions in the scene
        """
        # Get or create dome light
        light_prim_path = "/World/Light"
        if not omni.usd.get_context().get_stage().GetPrimAtPath(light_prim_path):
            omni.kit.commands.CreateLightCommand(
                light_type="DomeLight",
                prim_path=light_prim_path,
                name="dome_light"
            ).do()

        light_prim = omni.usd.get_context().get_stage().GetPrimAtPath(light_prim_path)

        # Randomize dome light color temperature (simulating different times of day)
        color_temps = [
            Gf.Vec3f(0.9, 0.85, 0.8),  # Warm (sunrise/sunset)
            Gf.Vec3f(0.95, 0.95, 1.0), # Neutral (noon)
            Gf.Vec3f(0.8, 0.9, 1.0),   # Cool (overcast)
            Gf.Vec3f(0.7, 0.8, 1.0)    # Cold (shade)
        ]

        color = random.choice(color_temps)
        light_prim.GetAttribute("inputs:color").Set(color)

        # Randomize intensity
        intensity = random.uniform(0.5, 1.5)
        light_prim.GetAttribute("inputs:intensity").Set(intensity * 1000)

        print(f"Lighting randomized - Color: {color}, Intensity: {intensity}")

    def randomize_object_positions(self):
        """
        Randomize positions of objects in the scene
        """
        for obj_prim_path in self.scene_objects:
            # Randomize position
            x = random.uniform(-4, 4)
            y = random.uniform(-4, 4)
            z = 0.1  # Slightly above ground

            # Randomize rotation
            rot_x = random.uniform(-0.2, 0.2)
            rot_y = random.uniform(-0.2, 0.2)
            rot_z = random.uniform(0, 2 * np.pi)  # Full rotation around Z

            omni.kit.commands.TransformMultiPrimsSRTCommand(
                prim_paths=[obj_prim_path],
                translation_offsets=[[x, y, z]],
                rotation_offsets=[[rot_x, rot_y, rot_z]],
                usd_context=omni.usd.get_context()
            ).do()

        print(f"Object positions randomized")

    def randomize_materials(self):
        """
        Randomize materials of objects in the scene
        """
        # This is a simplified example - in practice, you'd have more complex material randomization
        material_paths = [
            "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Blocks/Materials/block_material_01.mdl",
            "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Blocks/Materials/block_material_02.mdl",
            "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Blocks/Materials/block_material_03.mdl"
        ]

        for obj_prim_path in self.scene_objects:
            # For simplicity, we'll just change the color of existing materials
            # In a real implementation, you would assign different materials
            prim = omni.usd.get_context().get_stage().GetPrimAtPath(obj_prim_path)

            # Randomize color
            color = Gf.Vec3f(
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0)
            )

            print(f"Material for {obj_prim_path} randomized - Color: {color}")

    def randomize_all(self):
        """
        Apply all randomization techniques
        """
        self.randomize_lighting()
        self.randomize_object_positions()
        self.randomize_materials()

        print("Domain randomization applied to scene")

# Example usage
def run_domain_randomization():
    world = World.instance()

    # Define scene objects to randomize
    scene_objects = [f"/World/Object_{i}" for i in range(5)]

    # Create domain randomizer
    randomizer = DomainRandomizer(world, scene_objects)

    # Randomize the scene
    randomizer.randomize_all()

    return randomizer

# Usage example
if __name__ == "__main__":
    randomizer = run_domain_randomization()
```

## Exercises

1. **Advanced Domain Randomization**: Implement more sophisticated domain randomization techniques like weather effects or different surface materials
2. **Multi-Sensor Data Generation**: Generate synthetic data for multiple sensor types simultaneously (camera, LIDAR, IMU)
3. **Dataset Annotation**: Automatically generate ground truth annotations for the synthetic data (bounding boxes, segmentation masks)

## Troubleshooting Tips

- Ensure your NVIDIA GPU supports Isaac Sim and has the latest drivers
- Check that Omniverse is properly installed and configured
- Verify that Isaac Sim extensions are enabled in Omniverse Kit
- Monitor GPU memory usage as photorealistic rendering can be demanding
- Use Isaac Sim's built-in profiler to identify performance bottlenecks

## Learning Outcomes
By completing this lesson, you will understand:
- How to set up photorealistic simulation environments in Isaac Sim
- How to configure and use various sensor models for data generation
- How to implement domain randomization techniques for robust AI training
- How to generate synthetic datasets with ground truth annotations