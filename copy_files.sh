#!/bin/bash

# Set the source and destination directories
source_dir="./"
destination_dir="../TBOT/src/turtlebot3_simulations/turtlebot3_simulations/turtlebot3_gazebo"

# Iterate over source files and copy them to the destination
for file in ./level*/*.world; do
    level=$(basename "$(dirname "$file")")
    cp "$file" "$destination_dir/worlds/turtlebot3_${level}.world"
done

for file in ./level*/*_robot*.launch; do
    filename=$(basename "$file")
    cp "$file" "$destination_dir/launch/${filename}"
done

echo "Files copied successfully!"
