require 'yaml'
constants = YAML.load_file(File.join(__dir__, 'constants.yaml'))

PI = constants['pi']

WHEEL_MASS = constants['wheel_mass']
WHEEL_RADIUS = constants['wheel_radius']
WHEEL_THICKNESS = constants['wheel_thickness']

LINK1_MASS = constants['link1_mass']
LINK1_LENGTH = constants['link1_length']
LINK1_RADIUS = constants['link1_radius']

LINK2_MASS = constants['link2_mass']
LINK2_LENGTH = constants['link2_length']
LINK2_RADIUS = constants['link2_radius']

LOAD_MASS = constants['load_mass']
LOAD_RADIUS = constants['load_radius']
