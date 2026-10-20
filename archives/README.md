# Historical hardware archives

These files preserve earlier hardware associated with my friend Leo's [original opensurgbot project](https://github.com/lgabp1/opensurgbot). They document the project's origins and provide reference material for ongoing Arduino-based development.

The PCB and mechanical designs here are legacy items. The controller now uses Arduino, and a complementary PCB or Arduino shield is planned. These archives are not current manufacturing or wiring instructions.

| Archive | Contents | Original source |
| --- | --- | --- |
| [opensurgbot_board](opensurgbot_board/) | Earlier KiCad PCB design, production exports, and backups for the STM32-based setup | [Leo's opensurgbot_board](https://github.com/lgabp1/opensurgbot_board/) |
| [opensurgbot_3Dprinted](opensurgbot_3Dprinted/) | Earlier platform assemblies, parts, and component reference CAD | [opensurgbot_3Dprinted](https://github.com/suyu-lily/opensurgbot_3Dprinted), linked from the original project |

Original files, README content, and contributor attribution are preserved. The mechanical repository has its own contributors; inclusion here acknowledges its association with Leo's project without assigning sole authorship to Leo.

The board's original README records missing trace connections in revision vB-1.03. Retain that limitation when consulting the design.

## Historical STM32 firmware

The STM32 project was already removed from the working tree before this hardware archive was created. It has not been restored or included here. Historical firmware can be found in [the original STM32CubeIDE projects](https://github.com/lgabp1/opensurgbot_pipeline/tree/main/STM32CubeIDE_projects).

## Archive handling

Design files were moved without content changes, preserving their relative paths and CAD assembly references. Nested Git metadata was left at the original local folder locations and is not part of this archive.
