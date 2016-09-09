# A Multi-Agent System for Adaptive Control of a Flapping-Wing Micro Air Vehicle

## Source Code

Supplementary Material for a dissertation:
A Multi-Agent System for Adaptive Control of a Flapping-Wing Micro Air Vehicle by Michal Podhradsky

Please see http://podhrmic.github.io for more details

The directory is organized as follows:
- **controller_Overo** is a part of the Multi Agent System (MAS) that communicates with low level motor API. It runs on Gumstix Overo. To compile it, simply type `make` in the project root directory. Requires `api_v_0_13`
- **QTracker_GUI** combines GUI, the pose estimation system and the computer part of MAS. It uses Qt library and requires Qt Creator and OpenCV library to compile. The MAS portion can be seamlessly transferred to the Overo and being run there.
