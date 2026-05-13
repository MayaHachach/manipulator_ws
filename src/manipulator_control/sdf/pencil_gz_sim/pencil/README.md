# Pencil — Gazebo Sim Model
**For: Gazebo Harmonic / Ionic / Jetty (gz sim)**
Not compatible with Gazebo Classic (gazebo11).

---

## Files

```
pencil/
├── model.config       ← gz sim model metadata  (SDF 1.9)
├── model.sdf          ← pencil geometry + materials
└── pencil_world.sdf   ← demo world with two pencils
```

## Design

| Part          | Geometry                          | Colour        | Z range (m)     |
|---------------|-----------------------------------|---------------|-----------------|
| Graphite tip  | Cylinder (r=0.6 mm)               | Dark grey     | 0.000 → 0.011   |
| Wood sheath   | Cylinder (r=2.2 mm)               | Wood brown    | 0.011 → 0.021   |
| Hex body      | 6 box panels + cylinder fill      | Yellow        | 0.021 → 0.165   |
| Ferrule       | Cylinder (r=4.05 mm)              | Metallic silver | 0.165 → 0.175 |
| Eraser        | Cylinder (r=3.75 mm)              | Pink          | 0.175 → 0.190   |

**Total length: 190 mm · Hex body flat-to-flat: 7 mm**

Materials use plain RGBA (`<ambient>` / `<diffuse>` / `<specular>`) tuned for the
Ogre2 PBR renderer used by gz sim. No Ogre material scripts (those are classic only).

---

## Installation

### Option A — Drop into gz sim model path
```bash
cp -r pencil/ ~/.gz/models/
```

### Option B — Environment variable (recommended for ROS 2 packages)
```bash
# In your terminal or .bashrc:
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/absolute/path/to/parent/of/pencil/
```

---

## Quick test (standalone)
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)
gz sim pencil/pencil_world.sdf
```

---

## Using in your own world SDF

```xml
<!-- Pencil standing upright (tip down) -->
<include>
  <uri>model://pencil</uri>
  <name>pencil_1</name>
  <pose>1.0 0.5 0.001 0 0 0</pose>
</include>

<!-- Pencil lying on its side (along X-axis) -->
<include>
  <uri>model://pencil</uri>
  <name>pencil_2</name>
  <pose>0.095 0.0 0.004 0 1.5708 0</pose>
</include>
```

---

## ROS 2 + gz sim (ros_gz)

In your launch file, set the resource path before spawning:
```python
import os
from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    os.path.join(get_package_share_directory('your_pkg'), 'models')
)
```
Place the `pencil/` folder under `your_pkg/models/`.

---

## Making it dynamic

In `model.sdf`, change:
```xml
<static>true</static>
```
to:
```xml
<static>false</static>
```
The collision geometry is already defined for all parts.
Inertial values are set for a ~10 g pencil.

---

## Customisation

- **Colour**: Edit `<diffuse>` RGBA in the body panel visuals.
- **Scale**: Multiply all `<pose>` z-offsets and geometry dimensions by the same factor.
- **Add logo text**: Use a texture albedo map via `<material><pbr><metal><albedo_map>`.
