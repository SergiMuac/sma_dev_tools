import typer
from typing import List
from typing_extensions import Annotated
from invoke.exceptions import UnexpectedExit 
from sma.utils import console, resolve_connection, build_ros_command
from sma.config import config_manager  # Import config manager

app = typer.Typer(help="Remote ROS2 execution tools")


# --- COMMANDS ---
@app.command(name="add-ws")
def add_workspace(
    path: Annotated[str, typer.Argument(help="Remote path to workspace (e.g. /home/user/robot_ws)")],
    target: Annotated[str, typer.Option("--target", "-r", help="Registered Robot Name")],
):
    """
    Add an extra workspace to source before running commands.
    """
    if not config_manager.get_robot(target):
        console.print(f"[red]Error:[/red] Robot '{target}' not found. Please register it first using 'sma register'.")
        raise typer.Exit(code=1)

    success = config_manager.add_workspace(target, path)
    
    if success:
        console.print(f"[green]Success![/green] Added workspace '{path}' to robot '{target}'.")
        current_list = config_manager.get_workspaces(target)
        console.print(f"Current workspaces for {target}: {current_list}")
    else:
        console.print("[red]Failed to add workspace.[/red]")

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

    # Fetch extra workspaces if the target is a registered name
    extra_workspaces = config_manager.get_workspaces(target)

    # Build the command using the helper
    ros_cmd = build_ros_command(
        distro=distro,
        extra_workspaces=extra_workspaces,
        final_cmd=f"ros2 run {package} {node}"
    )

    console.print(f"[bold green]Running:[/bold green] ros2 run {package} {node}")
    if extra_workspaces:
        console.print(f"[dim]Including workspaces: {extra_workspaces}[/dim]")
    
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

    # Fetch extra workspaces
    extra_workspaces = config_manager.get_workspaces(target)

    # Build command
    ros_cmd = build_ros_command(
        distro=distro,
        extra_workspaces=extra_workspaces,
        final_cmd=f"ros2 launch {package} {file}"
    )

    console.print(f"[bold green]Launching:[/bold green] ros2 launch {package} {file}")
    if extra_workspaces:
        console.print(f"[dim]Including workspaces: {extra_workspaces}[/dim]")

    try:
        c.run(ros_cmd, pty=True)
    except KeyboardInterrupt:
        console.print("\n[yellow]Stopping remote launch...[/yellow]")
    except UnexpectedExit as e:
        console.print(f"[red]Launch stopped with error code {e.result.exited}[/red]")