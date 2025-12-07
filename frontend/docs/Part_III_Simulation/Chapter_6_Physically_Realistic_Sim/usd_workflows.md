---
sidebar_position: 2
title: "USD Workflows for Isaac Sim"
---

# USD Workflows for Isaac Sim

## Overview

Universal Scene Description (USD) is Pixar's scene description format that serves as the foundation for NVIDIA Isaac Sim. This chapter details the USD workflows essential for humanoid robotics simulation, including asset creation, scene composition, and integration with Isaac Sim's features.

## Understanding USD

### What is USD?

Universal Scene Description (USD) is:
- A file format and software development kit (SDK) for describing, reading, writing, and streaming 3D scenes
- A layered, composition-based system that enables collaborative workflows
- A format that supports animation, materials, lighting, and physics properties
- The primary scene representation format for NVIDIA Isaac Sim

### USD File Extensions

- `.usd` - ASCII text format (human-readable)
- `.usda` - ASCII text format (explicitly ASCII)
- `.usdc` - Compressed binary format
- `.usdz` - Zipped USD for sharing

## USD Structure for Robotics

### Basic USD Composition

A USD scene for robotics typically includes:

```
Root Stage
├── Robot Assets
│   ├── Base Link
│   ├── Joints
│   ├── Links
│   └── Sensors
├── Environment
│   ├── Ground Plane
│   ├── Objects
│   └── Lighting
├── Materials
│   ├── Robot Materials
│   └── Environment Materials
└── Physics Properties
    ├── Collision Shapes
    └── Joint Limits
```

### USD Prim Types for Robots

Key USD primitive types used in robotics:

```usda
# Example USD structure for a robot
def Xform "Robot" (
    prepend references = @RobotAsset.usd@
)
{
    # Robot links
    def Xform "BaseLink"
    {
        def Mesh "VisualMesh"
        {
            # Visual representation
        }
        def Mesh "CollisionMesh"
        {
            # Collision representation
        }
    }

    # Joints
    def Xform "LeftHip"
    {
        # Joint properties
    }

    # Sensors
    def Xform "Camera"
    {
        # Camera properties
    }
}
```

## Creating USD Assets for Humanoid Robots

### Robot Asset Structure

A well-structured robot USD asset should follow this hierarchy:

```usda
# Robot asset file (Robot.usd)
def Xform "BipedalRobot"
{
    # Metadata
    custom string robot:name = "bipedal_robot"
    custom double robot:mass = 50.0
    custom string robot:version = "1.0"

    # Base link
    def Xform "base_link"
    {
        def Mesh "visual"
        {
            # Visual mesh properties
            rel material:binding = </BipedalRobot/Materials/RobotBodyMaterial>
            points [ ... ]  # Vertex positions
            faceVertexIndices [ ... ]  # Face indices
            faceVertexCounts [ ... ]  # Face vertex counts
        }

        def Mesh "collision"
        {
            # Collision mesh properties
            points [ ... ]
            faceVertexIndices [ ... ]
            faceVertexCounts [ ... ]
        }

        # Mass and inertia properties
        add double mass:mass = 5.0
        add double3 mass:diagonalInertia = (0.1, 0.1, 0.1)
    }

    # Joint definitions
    def Xform "left_hip_yaw"
    {
        # Joint properties
        add double joint:lowerLimit = -1.57
        add double joint:upperLimit = 1.57
        add double joint:effortLimit = 100.0
        add double joint:velocityLimit = 3.0
    }
}
```

### Material Definitions

USD materials using MaterialX:

```usda
# Material definitions
def Material "Materials/RobotBodyMaterial"
{
    def Shader "PreviewSurface"
    {
        uniform token info_id = "UsdPreviewSurface"
        color3f inputs:diffuseColor = (0.7, 0.7, 0.7)
        float inputs:metallic = 0.1
        float inputs:roughness = 0.3
        float inputs:clearcoat = 0.0
        float inputs:clearcoatRoughness = 0.01
        float inputs:opacity = 1.0
        float inputs:ior = 1.5
        float inputs:specularColor = (1.0, 1.0, 1.0)
    }

    def Shader "Out"
    {
        uniform token info_id = "MaterialX"
        token inputs:surface.connect = </BipedalRobot/Materials/RobotBodyMaterial/PreviewSurface.outputs:surface>
    }
}
```

## USD Composition and Layering

### Layer Types in USD

USD uses a layer-based composition system:

1. **Root Layer**: The main file containing the primary scene
2. **Sublayers**: Additional layers composed into the root
3. **References**: Include external USD files
4. **Payloads**: Lazy-loaded content for large scenes

### Composition Example

```python
# Python example of USD composition
from pxr import Usd, Sdf, UsdGeom

def create_composed_robot_scene():
    """
    Create a composed USD scene with robot and environment
    """
    # Create root stage
    stage = Usd.Stage.CreateNew("ComposedScene.usd")

    # Add references to robot asset
    robot_prim = stage.DefinePrim("/Robot", "Xform")
    robot_prim.GetReferences().AddReference("Robot.usd")

    # Add references to environment
    env_prim = stage.DefinePrim("/Environment", "Xform")
    env_prim.GetReferences().AddReference("Environment.usd")

    # Override specific properties
    base_link = stage.GetPrimAtPath("/Robot/base_link")
    if base_link.IsValid():
        # Override position
        xform = UsdGeom.Xformable(base_link)
        xform.AddTranslateOp().Set((0, 0, 0.5))

    # Save the composed scene
    stage.GetRootLayer().Save()

    return stage
```

## Isaac Sim USD Extensions

### Isaac Sim-Specific Schemas

Isaac Sim adds specific schemas for robotics:

```usda
# Isaac Sim extensions
def Xform "Robot" (
    prepend references = @Robot.usd@
)
{
    # Isaac Sim physics properties
    def PhysicsRigidBodyAPI "base_link"
    {
        add double physics:mass = 5.0
        add double3 physics:centerOfMass = (0, 0, 0)
        add double3 physics:diagonalInertia = (0.1, 0.1, 0.1)
    }

    # Isaac Sim joint properties
    def PhysicsJointAPI "left_hip_yaw"
    {
        add double physics:lowerLimit = -1.57
        add double physics:upperLimit = 1.57
        add double physics:effort = 100.0
        add double physics:velocity = 3.0
    }

    # Isaac Sim sensor properties
    def Camera "camera"
    {
        add double horizontalAperture = 20.955
        add double verticalAperture = 15.2908
        add double focalLength = 300.0
    }
}
```

## USD Conversion Workflows

### URDF to USD Conversion

Converting URDF to USD for Isaac Sim:

```python
# Example URDF to USD conversion
import urdf2usd
from pxr import Usd, UsdGeom, Sdf

def convert_urdf_to_isaac_usd(urdf_path, usd_path):
    """
    Convert URDF to Isaac Sim compatible USD
    """
    # Use Isaac Sim's URDF converter
    urdf2usd.run(
        urdf_path=urdf_path,
        out_path=usd_path,
        merge_fixed_joints=False,
        convex_decomposition=False,
        material_path=None,
        usd_format='usd'
    )

    # Post-process to add Isaac Sim specific properties
    stage = Usd.Stage.Open(usd_path)

    # Add physics properties
    add_physics_properties_to_stage(stage)

    # Add material properties
    add_material_properties_to_stage(stage)

    # Save the modified USD
    stage.GetRootLayer().Save()

def add_physics_properties_to_stage(stage):
    """
    Add Isaac Sim physics properties to USD stage
    """
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xform):
            # Add rigid body properties
            add_rigid_body_properties(prim)

        elif prim.IsA(UsdGeom.Mesh):
            # Add collision properties
            add_collision_properties(prim)

def add_material_properties_to_stage(stage):
    """
    Add material properties to USD stage
    """
    # Create materials for the robot
    create_robot_materials(stage)

def create_robot_materials(stage):
    """
    Create and assign materials to robot parts
    """
    # Create material prim
    material_path = Sdf.Path("/Materials/RobotBodyMaterial")
    material = UsdShade.Material.Define(stage, material_path)

    # Create surface shader
    shader = UsdShade.Shader.Define(stage, material_path.AppendChild("PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.7, 0.7, 0.7))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)

    # Connect shader to material
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # Bind material to mesh
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            UsdShade.MaterialBindingAPI(mesh).Bind(material)
```

## USD Animation and Rigging

### Joint Animation in USD

For humanoid robots, USD can represent joint animations:

```usda
# Example joint animation
def Xform "BipedalRobot"
{
    def Xform "left_hip_yaw"
    {
        # Static joint properties
        add double joint:lowerLimit = -1.57
        add double joint:upperLimit = 1.57

        # Animated joint values
        float joint:position.timeSamples = {
            0: 0.0,
            1: 0.1,
            2: 0.0,
            3: -0.1,
            4: 0.0
        }
    }
}
```

### Walking Animation Example

```python
def create_walking_animation(stage):
    """
    Create a walking animation for the bipedal robot
    """
    # Define walking cycle (4 seconds)
    cycle_duration = 4.0
    steps_per_cycle = 20

    # Calculate time samples
    time_step = cycle_duration / steps_per_cycle

    for step in range(steps_per_cycle + 1):
        time = step * time_step

        # Calculate joint positions for walking gait
        left_hip_angle = 0.2 * math.sin(2 * math.pi * time / cycle_duration)
        right_hip_angle = 0.2 * math.sin(2 * math.pi * time / cycle_duration + math.pi)

        # Set joint positions at this time
        left_hip = stage.GetPrimAtPath("/Robot/left_hip_yaw")
        right_hip = stage.GetPrimAtPath("/Robot/right_hip_yaw")

        if left_hip.IsValid():
            joint_api = UsdPhysics.JointAPI(left_hip)
            # Set animated joint position

        if right_hip.IsValid():
            joint_api = UsdPhysics.JointAPI(right_hip)
            # Set animated joint position
```

## USD Variant Sets

### Using Variants for Different Configurations

USD variant sets allow multiple robot configurations in one file:

```usda
# Robot with different configurations
def Xform "BipedalRobot" (
    prepend variantSet = "configurations"
)
{
    variantSet "configurations" = {
        "default" = {
            def Xform "base_link" { }
            def Xform "left_leg" { }
            def Xform "right_leg" { }
        }
        "walking_gear" = {
            def Xform "base_link" { }
            def Xform "left_leg" {
                # With additional sensors
                def Xform "imu" { }
                def Xform "force_torque" { }
            }
            def Xform "right_leg" {
                # With additional sensors
                def Xform "imu" { }
                def Xform "force_torque" { }
            }
        }
        "dancing_gear" = {
            def Xform "base_link" { }
            def Xform "left_leg" {
                # With different joint limits
            }
            def Xform "right_leg" {
                # With different joint limits
            }
        }
    }
}
```

## USD for Domain Randomization

### Randomizable USD Properties

USD properties that can be randomized for domain randomization:

```python
def setup_domain_randomization(stage):
    """
    Setup USD properties for domain randomization
    """
    # Randomize material properties
    randomize_material_properties(stage)

    # Randomize lighting
    randomize_lighting_properties(stage)

    # Randomize object positions
    randomize_object_positions(stage)

def randomize_material_properties(stage):
    """
    Randomize material properties in USD
    """
    material_prims = [prim for prim in stage.Traverse() if prim.IsA(UsdShade.Material)]

    for material_prim in material_prims:
        material = UsdShade.Material(material_prim)

        # Get the surface shader
        surface_shader = material.GetSurfaceOutput().GetConnectedSource()[0]
        if surface_shader:
            # Randomize diffuse color
            random_color = (random.uniform(0.2, 0.8), random.uniform(0.2, 0.8), random.uniform(0.2, 0.8))
            surface_shader.GetInput("diffuseColor").Set(random_color)

            # Randomize roughness
            random_roughness = random.uniform(0.1, 0.9)
            surface_shader.GetInput("roughness").Set(random_roughness)

            # Randomize metallic
            random_metallic = random.uniform(0.0, 0.5)
            surface_shader.GetInput("metallic").Set(random_metallic)
```

## USD Tools and Utilities

### USD Python API

Common USD operations for robotics:

```python
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf

def analyze_robot_usd(usd_path):
    """
    Analyze robot USD file for robotics properties
    """
    stage = Usd.Stage.Open(usd_path)

    # Get robot links
    links = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform):
            if "link" in prim.GetName().lower():
                links.append(prim)

    # Get joints
    joints = []
    for prim in stage.Traverse():
        if "joint" in prim.GetName().lower() or "hip" in prim.GetName().lower() or "knee" in prim.GetName().lower():
            joints.append(prim)

    # Get sensors
    sensors = []
    for prim in stage.Traverse():
        if "camera" in prim.GetName().lower() or "lidar" in prim.GetName().lower() or "imu" in prim.GetName().lower():
            sensors.append(prim)

    return {
        'links': links,
        'joints': joints,
        'sensors': sensors,
        'total_prims': len(list(stage.Traverse()))
    }

def validate_robot_usd(stage):
    """
    Validate robot USD for Isaac Sim compatibility
    """
    issues = []

    # Check for required properties
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            if not mesh.GetPointsAttr().Get():
                issues.append(f"Mesh {prim.GetPath()} missing points")

        elif prim.IsA(UsdShade.Material):
            material = UsdShade.Material(prim)
            if not material.GetSurfaceOutput().GetConnectedSource():
                issues.append(f"Material {prim.GetPath()} missing surface shader")

    return issues
```

## Isaac Sim USD Best Practices

### 1. Efficient Asset Organization

- Use references for reusable components
- Organize assets in a hierarchical structure
- Use meaningful prim names
- Include metadata for identification

### 2. Performance Considerations

- Use appropriate level of detail
- Optimize mesh complexity
- Use instancing for repeated objects
- Minimize the number of materials

### 3. Compatibility

- Ensure USD files are compatible with Isaac Sim version
- Use supported prim types
- Validate physics properties
- Test in Isaac Sim environment

## Troubleshooting USD Issues

### Common USD Problems

1. **Invalid References**: Check file paths and existence
2. **Missing Materials**: Ensure materials are properly defined
3. **Physics Errors**: Validate mass and inertia properties
4. **Animation Issues**: Check time sampling and interpolation

### USD Validation

```python
def validate_usd_for_isaac_sim(usd_path):
    """
    Validate USD file for Isaac Sim compatibility
    """
    stage = Usd.Stage.Open(usd_path)

    if not stage:
        return False, "Could not open USD file"

    # Check for basic requirements
    issues = []

    # Check if there's at least one Xform
    xforms = [p for p in stage.Traverse() if p.IsA(UsdGeom.Xform)]
    if not xforms:
        issues.append("No Xform prims found - USD needs transform hierarchy")

    # Check for mesh geometry
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        issues.append("No Mesh prims found - robot needs visual geometry")

    # Check for materials
    materials = [p for p in stage.Traverse() if p.IsA(UsdShade.Material)]
    if not materials:
        issues.append("No Material prims found - consider adding materials")

    return len(issues) == 0, issues
```

## Integration with Isaac Sim Workflows

### Loading USD in Isaac Sim

```python
import omni
from pxr import Usd, Sdf

def load_robot_usd_in_isaac_sim(usd_path):
    """
    Load robot USD in Isaac Sim
    """
    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # Add reference to robot USD
    robot_prim = stage.DefinePrim("/World/Robot", "Xform")
    robot_prim.GetReferences().AddReference(usd_path)

    # Apply Isaac Sim specific properties
    apply_isaac_sim_properties(robot_prim)

def apply_isaac_sim_properties(robot_prim):
    """
    Apply Isaac Sim specific properties to robot
    """
    # Add physics properties
    add_physics_to_robot(robot_prim)

    # Add ROS bridge properties
    add_ros_bridge_properties(robot_prim)

    # Add sensor configurations
    add_sensor_configurations(robot_prim)
```

## Summary

USD workflows are fundamental to creating effective humanoid robotics simulations in Isaac Sim. By understanding USD's layered composition system, material definitions, and Isaac Sim-specific extensions, you can create sophisticated simulation environments that enable robust AI training and validation. The ability to randomize USD properties provides the domain randomization necessary for Sim-to-Real transfer learning, making USD a critical component in the development of advanced humanoid robots.