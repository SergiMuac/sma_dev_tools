import typer
# Import the commands/apps
from sma.modules.common.register import register
from sma.modules.common.list_robots import list_robots
from sma.modules.ros2.deploy import deploy_command
from sma.modules.ros2.ros2_cmd import app as ros2_app

app = typer.Typer(name="sma", add_completion=True, help="Robotics Development CLI")

# 2. Add commands to app
app.command()(register)
app.command(name="list")(list_robots)
app.command()(deploy_command)

# Add the ROS2 group
app.add_typer(ros2_app, name="ros2")

if __name__ == "__main__":
    app()