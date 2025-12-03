#!/bin/bash

uv run pyinstaller --onefile src/sma/main.py \
    --name sma \
    --paths src \
    --hidden-import=shellingham.posix \
    --collect-all=cryptography \
    --collect-all=paramiko \
    --collect-all=fabric