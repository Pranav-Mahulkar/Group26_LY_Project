# Group26_LY_Project

This project, IMU-Based Gait Analysis Using Machine Learning, addresses the need for a localized, cohesive system capable of analyzing human gait using IMU (Inertial Measurement Unit) sensors. It leverages a machine learning approach to analyze movement data collected from wearable IMU sensors placed at key points on the body. With the help of a Raspberry Pi, this setup enables healthcare professionals to monitor patient progress effectively, allowing for more precise treatment and rehabilitation tracking.

The system’s primary objective is to create a unified platform that can:
- Collect gait data accurately through IMU sensors.
- Analyze this data using machine learning models that identify subtle gait patterns and abnormalities.
- Provide healthcare professionals with actionable insights to make informed treatment decisions.

A significant gap exists in the availability of tools that integrate IMU sensor data acquisition with advanced analysis for healthcare applications. Traditional gait analysis systems often lack local availability, are high-cost, or are not user-friendly for daily clinical usage. Our project proposes an affordable, portable solution, combining the flexibility of Python programming on a Raspberry Pi with OpenSim, a simulation software, to visualize patient gait on a musculoskeletal model. 

This system’s functionality is supported by multiple components:
- **Hardware**: The IMU sensors gather accelerometer and gyroscope data, connected to a Raspberry Pi for processing and data storage.
- **Software**: The core of the system is a Python-based codebase, which integrates filtering algorithms for noise reduction, machine learning for data analysis, and OpenSim for simulation.
- **Security and Confidentiality**: Collected data is securely stored, ensuring patient privacy with encryption and access control mechanisms.

In summary, this project offers a scalable, modular, and cost-effective solution that can benefit clinics and rehabilitation centers by making gait analysis more accessible and practical. Through IMU-based data capture and real-time analysis, the system aims to improve healthcare professionals' ability to track recovery progress, personalize treatment, and make data-driven decisions.
