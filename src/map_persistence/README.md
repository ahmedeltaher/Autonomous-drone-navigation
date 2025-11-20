# Map Persistence

**Feature ID: 1.1.9 - Map Persistence**  
**Priority: P1**

Save and load occupancy grid maps for repeated missions.

---

## 📋 Overview

Persistent map storage with:
- Automatic saving (every 60s)
- Manual save/load services
- YAML metadata
- NumPy serialization

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select map_persistence

# Launch
ros2 launch map_persistence persistence.launch.py

# Save map manually
ros2 service call /save_map std_srvs/srv/Trigger

# Load map
ros2 service call /load_map std_srvs/srv/Trigger
```

---

## 📊 Topics & Services

### Input
- `/static_map` - Map to save

### Output
- `/loaded_map` - Loaded map

### Services
- `/save_map` - Save current map
- `/load_map` - Load latest map

---

## ⚙️ Parameters

```yaml
map_directory: 'maps'   # Storage location
auto_save: true         # Enable auto-save
save_interval: 60.0     # Save every 60s
```

---

## 🎯 Storage Format

**Files**:
- `map_YYYYMMDD_HHMMSS.npy` - Map data
- `map_YYYYMMDD_HHMMSS.yaml` - Metadata

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
