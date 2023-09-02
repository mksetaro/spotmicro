# Spot Marley
Yet another spot micro project

### Requirements
```
bazel 6.3.0
```

## Get Started
Launch script to setup project:

```
./setup_project.sh
```
Script content:

- Install bazelisk [TODO]
- It adds environmental variables to bashrc/zshrc

### Tools
```
bazel run //tools:buildifier #runs bazel buildifier
bazel run //tools:py_lint #runs python linter
```
### Simulation
```
bazel run //simulation:example #runs pybullet example
```