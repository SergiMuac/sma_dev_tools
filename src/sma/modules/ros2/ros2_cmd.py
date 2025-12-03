import typer
from typing_extensions import Annotated
from invoke.exceptions import UnexpectedExit 
from sma.utils import console, resolve_connection

# Create sub-app
app = typer.Typer(help="Remote ROS2 execution tools")

@app.command()
def run(
    package: Annotated[str, typer.Argument()],
    node: Annotated[str, typer.Argument()],
    target: Annotated[str, typer.Option("--target", "-r", "--ip", help="Robot Name OR IP")],
    user: Annotated[str, typer.Option()] = "user",
    distro: Annotated[str, typer.Option()] = "humble",
):
    """Run a specific ROS2 node."""
    if not target:
        console.print("[red]Error:[/red] You must specify --target, -r, or --ip")
        raise typer.Exit(code=1)

    c = resolve_connection(target, user)

    ros_cmd = (
        f"bash -c 'source /opt/ros/{distro}/setup.bash && "
        f"source dev_ws/install/setup.bash && "
        f"ros2 run {package} {node}'"
    )

    console.print(f"[bold green]Running:[/bold green] ros2 run {package} {node}")
    try:
        c.run(ros_cmd, pty=True)
    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping remote node...[/yellow]")
    except UnexpectedExit as e:
        console.print(f"[red]Node stopped with error code {e.result.exited}[/red]")

@app.command()
def launch(
    package: Annotated[str, typer.Argument()],
    file: Annotated[str, typer.Argument()],
    target: Annotated[str, typer.Option("--target", "-r", "--ip", help="Robot Name OR IP")],
    user: Annotated[str, typer.Option()] = "user",
    distro: Annotated[str, typer.Option()] = "humble",
):
    """Launch a ROS2 launch file."""
    if not target:
        console.print("[red]Error:[/red] You must specify --target, -r, or --ip")
        raise typer.Exit(code=1)

    c = resolve_connection(target, user)

    ros_cmd = (
        f"bash -c 'source /opt/ros/{distro}/setup.bash && "
        f"source dev_ws/install/setup.bash && "
        f"ros2 launch {package} {file}'"
    )

    console.print(f"[bold green]Launching:[/bold green] ros2 launch {package} {file}")
    try:
        c.run(ros_cmd, pty=True)
    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping remote launch...[/yellow]")
    except UnexpectedExit as e:
        console.print(f"[red]Launch stopped with error code {e.result.exited}[/red]")