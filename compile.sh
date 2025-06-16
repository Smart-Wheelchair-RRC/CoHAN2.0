# Update the submodules
git submodule update --init --recursive

type=${1:-Release}

# Build the selected packages (for development)
if command -v catkin &> /dev/null
then
    catkin build -j4 -DCMAKE_BUILD_TYPE=$type -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --cmake-args -Wno-dev
else
    echo "catkin installation not found!"
    exit 1
fi
cd $PWD/build/ && jq -s add **/*.json > compile_commands.json
echo "Combined the JSON"
