#!/bin/bash

ENV_NAME=Pheno

PYTHON_SCRIPT=./evaluate.py

conda activate $ENV_NAME

python $PYTHON_SCRIPT
