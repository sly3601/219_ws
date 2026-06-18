import tensordict
print("tensordict preload ok")

import rsl_rl
print("rsl_rl preload ok")

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

print("simulation app ok after preload")

simulation_app.close()
