ulimit -n 4096
python3 config.py
python3 env_modified.py -n 1
python3 generateSuperEnvironments.py
python3 generate_envs.py