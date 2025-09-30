# Neural-ATTF-Simulator
This is the Python simulator branch. This branch provides a folder with a pipeline for the Neural ATTF simulations using Robotarium Python simulator.

## Test Platform
1. Hardware: Windows 11
2. Python 3.11.9

## Prerequisites
1. Install [Robotarium Python simulator](https://github.com/robotarium/robotarium_python_simulator).
2. Copy `my_tests` folder from this repository inside the Robotarium Python simulator. The simulator directory would appear as below:
  ```
  robotarium_python_simulator/
  ├── my_tests/
  │ ├── config.yaml
  │ ├── eng.json
  │ ├── generate_submission.py
  │ ├── local_script_test1.py
  │ ├── schedule.json
  │ ├── utility.py
  │ └── warehouse_image.png
  ├── robotarium_python_simulator.egg-info
  ├── rps
  ├── LICENSE
  ├── README.md
  └── setup.py
  ```

## Usage
1. File description:
  1. `config.yaml`: Contains the tunable parameters for the experiments.
