# Open Source Constant Current Spinal Cord Stimulator with Wireless Communications Features

## Background

This repository contains the code developed for my Honours thesis at The University of Sydney. It uses code developed by Dr. Monzurul Alam for the spinal pulse signal, and adds wireless communications and a GUI. The stimulator was implemented using Arduino and the GUI was designed in Python.

## Thesis - Remotely-Controlled Spinal Cord Stimulation via Message Query Telemetry Transport

Degree: Bachelor of Engineering (Honours) (Electrical Engineering), The University of Sydney

Supervisor: Professor Alistair McEwan

### Abstract 
Spinal cord injury is a highly debilitating condition that affects millions around the world. Daily life becomes difficult for those who lose limb function or experience chronic pain caused by a damaged spinal cord. These conditions make it difficult for patients to visit doctors for regular treatments. Thus, there has been increasing interest in designing medical devices that can connect and be controlled through the internet. These devices accelerate healthcare processes by providing real-time monitoring and updates of patient conditions. Most crucially, these devices can notify doctors and patients and prompt early intervention through computational data analysis, such as heart rate or blood pressure detection. This thesis focuses heavily on the development of software and firmware for controlling a constant-current stimulator device via the internet. The communications protocol adopted is Message Query Telemetry Transport (MQTT), which is lightweight, scalable and easy to understand. Each client is connected to a server called a broker. The clients can subscribe to specific message topics, which the broker will publish for the clients to read. The software for controlling the stimulation parameters is written in Python, using the Tkinter library for GUI development. The firmware for the stimulating device is written in Arduino, and is inherited from an open source project, OpenIStim, with additions for MQTT connection capabilities. The results show a latency of 1-2s, an output resolution of 1ms, and successful management of boundary and error conditions. Future work should focus on reducing the latency and increasing the output resolution. Compared to other devices published in the literature, this device is not fully functional, but lays the groundwork for future development of open-source medical device development.

## Links

OpenIStim Original Code: https://github.com/MonzurulAlam/OpenIstim
