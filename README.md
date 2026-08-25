# Drone Swarm Simulator - GUI & Renode Emulation

This repository contains a high-fidelity Software-in-the-Loop (SITL) Drone Swarm Simulator.
It leverages **Renode** to run real, unmodified flight controller firmware compiled for ARM Cortex-M targets (e.g., STM32/Pixhawk `.elf` binaries) cycle-by-cycle, and provides a Ground Control Station (GCS) built in **PySide6**.

## Architecture Overview

### 1. Hardware Emulation (Renode Backend)
- **`platforms/`**: Contains `.repl` definitions for the Cortex-M targets, mapping CPU, NVIC, Memory, UARTs, and sensor buses.
- **`backend/renode_orchestrator.py`**: Manages the headless Renode instance, instantiates virtual machines (VMs), and bridges their UARTs to host UDP ports for telemetry.
- **`backend/isr_manager.py`**: Emulates hardware fault injection. It can freeze the VM, take a snapshot of CPU registers (`PC`, `SP`, `LR`, `R0-R12`), inject hardware interrupts via the virtual NVIC, and restore the state seamlessly.
- **`backend/sensor_feeder.py`**: A synthetic sensor feeder that writes mock spatial data (IMU, Barometer, GPS) directly into Renode's virtual sensor memory to prevent firmware failsafes.
- **`backend/logger.py`**: Centralized logger outputting to `logs/backend_emulation.log`.

### 2. Ground Control Station (GUI)
- **`services/api_client.py`**: Uses PySide6's `QUdpSocket` to listen for telemetry from the Renode UART bridges and dispatch hardware ISR commands without blocking the UI thread.
- **`gui/`**: PySide6 dashboard widgets (`main_window.py`, `telemetry_dashboard.py`, `map_viewer.py`) displaying real-time telemetry and providing controls to trigger and restore hardware ISRs.
- **`contracts/gui_orchestration.py`**: Defines the shared Pydantic models for UDP telemetry datagrams and GUI-to-emulator command schemas.

## Requirements
- Python 3.9+
- `PySide6 >= 6.7`
- `pydantic >= 2.0`
- `pytest`
- (Optional but required for real emulation) **Renode**

Install dependencies using:
```bash
pip install -r requirements.txt
```

## Running the Application

1. **Start the Renode Backend Orchestrator**:
   This runs the local UDP servers for orchestration and telemetry generation.
   ```bash
   python backend/renode_orchestrator.py
   ```

2. **Launch the GUI**:
   In a separate terminal, launch the PySide6 dashboard.
   ```bash
   python main.py
   ```

3. **Interact**:
   Use the toolbar to select a SysID (e.g., 1 or 2) and click **Inject Hardware ISR**. The backend will snapshot the CPU and inject the interrupt. Click **Restore State** to resume normal operation.

## Testing
Run unit tests with pytest:
```bash
pytest tests/
```
