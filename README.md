# opensurgbot

This repository continues development from [Leo's original opensurgbot project](https://github.com/lgabp1/opensurgbot).

## About

opensurgbot aims to be an open-source, low-cost, easy-to-use and easy-to-improve surgical robot platform, compatible with the [SurRoL](https://github.com/med-air/SurRoL) framework.

The first prototype has been successfully built and tested.

## Current hardware and archives

The current controller uses Arduino. A complementary PCB, potentially an Arduino shield, is planned. The old PCB and 3D-printed designs are preserved in [archives/](archives/README.md) as historical references, separate from the updated hardware.

- These archives preserve hardware from my friend Leo's [original opensurgbot project](https://github.com/lgabp1/opensurgbot) and its associated repositories. Original contributor credits and documentation are retained; see the archive index for source links.
- [Archived PCB design](archives/opensurgbot_board/): the earlier board used with the STM32-based setup.
- [Archived mechanical design](archives/opensurgbot_3Dprinted/): the earlier platform and component CAD files.

The archived files are not the current Arduino hardware or manufacturing instructions.

## Repositories

| Repository | Description |
|------------|-------------|
| [opensurgbot](https://github.com/lgabp1/opensurgbot) | opensurgbot root repository |
| (no link yet) | Documentation repository  |
| [opensurgbot_SurRoL](https://github.com/lgabp1/opensurgbot_SurRoL/) | Integration of opensurgbot with the SurRoL framework |
| [opensurgbot_pipeline](https://github.com/lgabp1/opensurgbot_pipeline/) | High level python api and microcontroller code |
| [opensurgbot_kinevizu](https://github.com/lgabp1/opensurgbot_kinevizu/) | 3D visualization of the da Vinci Large Needle Drive and kinematics model |
| [opensurgbot_board](https://github.com/lgabp1/opensurgbot_board/) | PCB design files for the opensurgbot custom board |
| [opensurgbot_3Dprinted](https://github.com/suyu-lily/opensurgbot_3Dprinted) | 3D printed parts of the opensurgbot platform |
| [opensurgbot/web](https://github.com/lgabp1/opensurgbot/tree/master/web) | GitHub page of the project |

## Installation and usage

- Current Arduino firmware: [ARDUINO_PIPELINE_FIRMWARE.ino](ARDUINO_PIPELINE_FIRMWARE/ARDUINO_PIPELINE_FIRMWARE.ino).
- Bench diagnostics: [ARDUINO_COMBINED_CHECK.ino](ARDUINO_COMBINED_CHECK/ARDUINO_COMBINED_CHECK.ino).
- For historical STM32 and first-prototype hardware references, see the [archives index](archives/README.md).

As for the software,
- Please refer to [opensurgbot_pipeline](https://github.com/lgabp1/opensurgbot_pipeline/) to use the high level python API.
- Please refer to [opensurgbot_SurRoL](https://github.com/lgabp1/opensurgbot_SurRoL/) for the integration with the SurRoL framework.
