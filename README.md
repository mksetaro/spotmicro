# Spot Marley
Yet another spot micro project

### Requirements
```
bazel 6.3.0
```

## Get Started

### Tools
```
bazel run //tools:buildifier #runs bazel buildifier
bazel run //tools:py_lint <optional-absolute-path-if-no-ws_root_env_variable-set> #runs black fix (be smart, set SPOT_MARLEY_WS_ROOT in your .bashrc/.zshrc/.whateverrc)
```
### Simulation
```
bazel run //simulation:example #runs pybullet example
```