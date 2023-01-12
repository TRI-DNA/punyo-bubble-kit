#!/bin/bash
# MicroXRCEAgent arguments
# -v <val>: 6 is very verbose, but is useful to get confirmation that pressure data is flowing. 
#
# -r <refs>: Required, as the client on the microcontroller (MC) is using the references method to create entities.
#            While the xml method could be used to handle the entities, it is only possible to set an interface whitelist via refs.
#            For more details on the refs format, see https://micro.ros.org/docs/tutorials/advanced/create_dds_entities_by_ref/
#            The <refs> file must contain the correct MC serialnumber. 
#            You can see the serialnumber in the log if there is a mismatch and/or check the devices under /dev/serial/by-id
# --dev <device>: Instead of /dev/ttyACMx, you may use /dev/serial/by-id/ or by-path to be more specific about the source
#
MicroXRCEAgent serial --dev /dev/ttyACM1 -b 115200 -r ./A1208.ref -v 6
