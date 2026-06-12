echo "Copying files into prius_launch config"
cp config/settings.yaml ../../prius_launch/launch/control/roadmap/settings.yaml
cp launch/roadmap.launch ../../prius_launch/launch/control/roadmap/roadmap.launch
sed -i '/  map_package_name:/ c\  map_package_name: "prius_launch"' ../../prius_launch/launch/control/roadmap/settings.yaml
sed -i '/  map_file_name: "maps/ c\  map_file_name: "launch/control/roadmap/maps/test_map.yaml"' ../../prius_launch/launch/control/roadmap/settings.yaml