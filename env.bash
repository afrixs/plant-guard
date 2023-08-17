# You can source this file in your ~/.bashrc to make loading pg_ws environment easier (via 'r2pg' command)
PROJECT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

alias r2pg="source $PROJECT_DIR/pg_ws/install/setup.bash"
