import typer
from typing_extensions import Annotated
from fabric import Connection
from paramiko.ssh_exception import AuthenticationException
from sma.config import config_manager
from sma.utils import console

# app = typer.Typer(help="Registration commands")

# @app.command()
def register(
    name: Annotated[str, typer.Option(help="Friendly name for the robot")],
    ip: Annotated[str, typer.Option(help="IP address")],
    user: Annotated[str, typer.Option(help="SSH Username")] = "user",
):
    """Register a robot to the local config."""
    console.print(f"Registering [cyan]{name}[/cyan] ({user}@{ip})...")
    
    save_pass = typer.confirm("Do you want to save the password?", default=False)
    password = None
    if save_pass:
        password = typer.prompt("Enter SSH Password", hide_input=True)

    try:
        if password:
            c = Connection(host=ip, user=user, connect_kwargs={"password": password})
            c.run("echo 'Test'", hide=True)
        else:
            c = Connection(host=ip, user=user)
            c.run("echo 'Test'", hide=True) 
            
        config_manager.register_robot(name, ip, user, password)
        console.print(f"[bold green]Success![/bold green] Robot '{name}' registered.")
        
    except AuthenticationException:
        console.print("[bold red]Authentication failed.[/bold red]")
    except Exception as e:
        console.print(f"[red]Could not connect: {e}[/red]")
