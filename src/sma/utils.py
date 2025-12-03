import typer
import tarfile
from pathlib import Path
from fabric import Connection
from paramiko.ssh_exception import AuthenticationException
from rich.console import Console

# Import the config instance
from sma.config import config_manager

# Shared Console object
console = Console()

def create_tarball(source_dir: Path, output_filename: str):
    with tarfile.open(output_filename, "w:gz") as tar:
        tar.add(source_dir, arcname=source_dir.name)

def _connect_with_auth_logic(ip: str, user: str) -> Connection:
    """Internal helper to handle keys vs password prompt"""
    c = Connection(host=ip, user=user)
    try:
        c.run("echo 'Check'", hide=True)
        return c
    except AuthenticationException:
        console.print("[yellow]SSH Key authentication failed.[/yellow]")
        password = typer.prompt("Enter SSH Password", hide_input=True)
        return Connection(host=ip, user=user, connect_kwargs={"password": password})
    except Exception as e:
        console.print(f"[bold red]Connection Error:[/bold red] {e}")
        raise typer.Exit(code=1)

def resolve_connection(target: str, user_opt: str = "user") -> Connection:
    """
    Determines if 'target' is a registered robot name or a raw IP.
    """
    robot_cfg = config_manager.get_robot(target)

    if robot_cfg:
        ip = robot_cfg["ip"]
        user = robot_cfg["user"]
        saved_pass = robot_cfg.get("password")
        
        console.print(f"[dim]Detected saved robot: [bold cyan]{target}[/bold cyan] ({user}@{ip})[/dim]")
        
        if saved_pass:
            return Connection(host=ip, user=user, connect_kwargs={"password": saved_pass})
        else:
            return _connect_with_auth_logic(ip, user)
    else:
        # Raw IP
        return _connect_with_auth_logic(target, user_opt)