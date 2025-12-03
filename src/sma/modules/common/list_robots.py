import typer
from typing_extensions import Annotated
from fabric import Connection
from paramiko.ssh_exception import AuthenticationException
from sma.config import config_manager
from sma.utils import console

# app = typer.Typer(help="Registration commands")

# @app.command(name="list")
def list_robots():
    """List registered robots."""
    data = config_manager.load()
    if not data:
        console.print("No robots registered.")
        return
    for name, details in data.items():
        console.print(f"[bold cyan]{name}[/bold cyan]: {details['user']}@{details['ip']}")