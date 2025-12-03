import typer
import os
import tempfile
from pathlib import Path
from typing_extensions import Annotated
from invoke.exceptions import UnexpectedExit 
from rich.progress import Progress, SpinnerColumn, TextColumn
from sma.utils import console, create_tarball, resolve_connection

def deploy_command(
    package_path: Annotated[Path, typer.Argument(
        exists=True, file_okay=False, dir_okay=True, readable=True
    )],
    target: Annotated[str, typer.Argument(help="Robot Name OR IP Address")],
    user: Annotated[str, typer.Option(help="Override user if using raw IP")] = "user",
):
    """Deploy and Build a ROS2 package."""
    c = resolve_connection(target, user)
    
    remote_src = "dev_ws/src"
    remote_ws = "dev_ws"
    package_name = package_path.name

    with tempfile.TemporaryDirectory() as tmp_dir:
        tar_name = f"{package_name}.tar.gz"
        local_tar_path = os.path.join(tmp_dir, tar_name)
        with console.status(f"[bold green]Compressing {package_name}..."):
            create_tarball(package_path, local_tar_path)
        
        try:
            with Progress(
                SpinnerColumn(), TextColumn("[progress.description]{task.description}"), console=console
            ) as progress:
                t1 = progress.add_task("Cleaning & Uploading...", total=None)
                c.run(f"mkdir -p {remote_src}", hide=True)
                c.run(f"rm -rf {remote_src}/{package_name}", hide=True)
                c.put(local_tar_path, remote=f"{remote_src}/{tar_name}")
                progress.update(t1, completed=1, description="[green]Files uploaded")

                t2 = progress.add_task("Extracting...", total=None)
                c.run(f"tar -xzf {remote_src}/{tar_name} -C {remote_src}", hide=True)
                c.run(f"rm {remote_src}/{tar_name}", hide=True)
                progress.update(t2, completed=1, description="[green]Extracted")

                t3 = progress.add_task(f"Building {package_name}...", total=None)
                build_cmd = (
                    f"bash -lc 'cd {remote_ws} && "
                    f"colcon build --symlink-install --packages-select {package_name}'"
                )
                c.run(build_cmd, hide=True)
                progress.update(t3, completed=1, description="[green]Build complete")

            console.print(f":rocket: [bold green]Deployed & Built on {c.host}[/bold green]")

        except UnexpectedExit as e:
            console.print("\n[bold red]Build Failed![/bold red]")
            print(e.result.stderr)
            raise typer.Exit(code=1)