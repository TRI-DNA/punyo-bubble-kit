import re
import subprocess

def get_mcu_id(device="/dev/ttyACM0"):
    """Returns the manufacturer serial number. """
    command = f"udevadm info --name={device}"
    output = subprocess.run(command.split(' '), capture_output=True, text=True)

    # Let's match on the ID_SERIAL_SHORT to get the unique id of the MCU
    # E: ID_SERIAL_SHORT=35A4E5A250555733362E3120FF091A1E"
    p = re.compile(".*ID_SERIAL_SHORT=(.*?)\n", flags = re.DOTALL)
    m = p.match(output.stdout)
    assert m is not None, f"Can not find a device on {device}"

    return m.group(1)


mcu_id = get_mcu_id()

# Read in the ros reference file
with open('ros.ref', 'r') as file :
  filedata = file.read()

# Get the mcu id in the current file
ros_ref_id = next(iter(re.findall("[\dA-Y]{32}", filedata)), None)
print(f"Updating bubble_{ros_ref_id} to bubble_{mcu_id}")

# Replace the mcu id
filedata_updated = re.sub('bubble_[\dA-Y]{32}', f"bubble_{mcu_id}", filedata, flags = re.M)

# Write out updated ros ref file.
with open('ros.ref', 'w') as file:
  file.write(filedata_updated)

