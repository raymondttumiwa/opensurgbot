# opensurgbot_kinevizu

Submodule of [opensurgbot](https://github.com/lgabp1/opensurgbot).

Kinematic model and kinematic visualization for the da Vinci Large Needle Driver (Model No. 420006).

<img width="1974" height="1297" alt="image" src="https://github.com/user-attachments/assets/d97d6971-d3dd-48d1-948c-ea25d1c76c79" />


## Note

This repository should probably be considered deprecated in the long term. Kinevizu should be seen as a repository related to experimenting with making a kinematic model since here the study is solely limited to the da Vinci large needle driver 420006

Later revisions of opensurgbot should generalize the idea: provide a modular way of developping a kinematic model used in KinematicDescription class of opensurgbot_pipeline python side. 

## Repository content

* [kinematics_study/KinevisuReport.pdf](./kinematics_study/KinevisuReport.pdf) provides some details about the kinematic model used
* [visualization/](./visualization/) provides a python-based 3D visualization of the proposed model

## KinevisuReport

A short report on the kinematic model.

## Visualization

A basic 3D matplotlib application to help verifying the model.

### Requirements

* [python 3.7+](https://python.org/)
* [numpy](https://numpy.org/) compatible with given python version
* [matplotlib](https://matplotlib.org/) compatible with given python version
* [pillow](https://github.com/python-pillow/Pillow) compatible with given python version

### Usage

Simply run the provided script with python `python kinevisu.py`, or import the `OpensurgbotViz` class in an external script.

# License

MIT License, see [LICENSE](./LICENSE) file for details.
