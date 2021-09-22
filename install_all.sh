# Fetching the catkin submodule
git submodule update --quiet --init --recursive --jobs 4 || true
git submodule sync --recursive
git submodule update --init --recursive --jobs 4


