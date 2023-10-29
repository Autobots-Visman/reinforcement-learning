#!/usr/bin/env bash
set -ex

# cd to the directory of this script
cd "$(dirname "${BASH_SOURCE[0]}")"

# TODO: poetry isn't playing nicely and installing to the system, so we need
# to call this via poetry for now.
# https://fastapi.tiangolo.com/tutorial/first-steps/
poetry run uvicorn app:app --reload --port 8000 --host 0.0.0.0
