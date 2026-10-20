# opensurgbot_SurRoL

Submodule of [opensurgbot](https://github.com/lgabp1/opensurgbot).

python library allowing [SurRoL](https://med-air.github.io/SurRoL/) tasks to drive the opensurgbot hardware. 

## Requirements

* [python 3.7+](https://www.python.org/)
* [SurRoL v1](https://github.com/med-air/SurRoL)
* My [opensurgbot_pipeline](https://github.com/lgabp1/opensurgbot_pipeline) library. It is acessible as a submodule, so one can clone this repository using `git clone ---recursive https://github.com/lgabp1/opensurgbot_SurRoL.git`

## Repository content and usage

The repository currently is quite messy, the most relevant elements are:
- [opensurgbot_pipeline/](./opensurgbot_pipeline): Contains the opensurgbot pipeline-related definitions.
- [opensurgbot_surrol_direct_injection.py](./opensurgbot_surrol_direct_injection.py): A simple example hack to allow a SurRoL task (here `NeedlePick`) to drive the opensurgbot hardware. See more details below. Basically, a procedure example to directly inject the opensurgbot pipeline into a SurRoL task.
- [opensurgbot_surrol_injector.py](./opensurgbot_surrol_injector.py): Defines a more complex class (probably harder to maintain) which uses python tricks to allow using the opensurgbot platform with any SurRoL task that uses PSM arms.

## Integration

To set up the opensurgbot pipeline and hardware, please refer to the [opensurgbot_pipeline](https://github.com/lgabp1/opensurgbot_pipeline) repository.

## Licence

(todo)