# 24GHz mmWave Sensor for XIAO (ESP-IDF Component)

A robust ESP-IDF component for interfacing with a 24GHz mmWave sensor on the XIAO platform. This project was born out of the need for a more reliable, efficient, and feature-rich alternative to the official Arduino version, which from my testing appears to suffer from issues like kernel panics and crashes after extended use.

## Overview

This component leverages the full power of the ESP-IDF framework and FreeRTOS to manage sensor communications efficiently via tasks and queues. With direct access to ESP-IDF's UART driver, it offers significantly enhanced performance and stability compared to the traditional SoftwareSerial used in the Arduino version.

## Features

Full Feature Support: Provides complete support for the 24GHz mmWave sensor's capabilities.  
Enhanced Stability: Avoids the pitfalls of the official Arduino version, such as kernel panics and long-term crashes.  
Efficient Communication: Uses ESP-IDF's robust UART driver along with FreeRTOS tasks and queues for optimized data handling.  
Testable: Comprehensive testing is built in. Easily run tests with the provided commands.  


## Installation

In your `idf_component.yaml` add following dependency:
```YAML
xiao_mmwave:
    git: "https://github.com/cichacz/xiao_mmwave.git"
    version: "main"
```

## Usage

In your main application or any component, include the header file:

```C
#include "xiao_mmwave.h"
```

Initialize the sensor. The only thing that it needs is the callback for status updates:

```C
void sensor_status_handler(radar_status_t *status)
{
    ESP_LOGD(TAG, "Read occupancy sensor status: %d, distance: %d, moving distance: %d, moving energy: %d, stationary distance: %d, stationary energy: %d", status->target_status, status->detection_distance, status->moving_target_distance, status->moving_target_energy, status->stationary_target_distance, status->stationary_target_energy);
}

void app_main(void)
{
    // Initialize sensor
    xiao_mmwave_init(sensor_status_handler);
}
```

## Testing

To build and run the tests:

Navigate to the test_app directory:
```sh
cd test_app
```
Build the project:
```sh
idf.py build
```
Monitor the output:
```sh
idf.py monitor
```

## Contributing

Contributions are welcome! Please feel free to open issues or submit pull requests for any improvements or bug fixes.

## License

This project is licensed under the MIT License.