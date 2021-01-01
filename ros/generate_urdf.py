import os
import xacro

os.environ['LC_NUMERIC'] = "en_US.UTF-8" # Fix for URDF not showing correctly in RViz

xacro_file = os.path.join(
    'src', 'clumsybot_description',
    'urdf',
    'clumsybot.urdf.xacro')

urdf_contents = xacro.process_file(xacro_file).toprettyxml(indent='  ')
print("URDF robot description file :" + xacro_file)
print(urdf_contents)

with open('robotmodel.xml', 'w') as desc_file:
    desc_file.write(urdf_contents)
